"""目标检测的封装层。

把 ``ultralytics`` 的输出转成本项目自己的 ``Detection2D``，让上层代码不必
到处出现 ``results[0].boxes.xyxy.cpu().numpy()`` 这类东西。

设计要点：

* **惰性导入 ultralytics** —— 这个包只在本模块被真正用到时才 import。
  于是 ``g2_core`` 的其余模块（frontier / projector / 状态机…）的单元测试
  不需要装 torch 就能跑。torch 是几 GB 的依赖，不该成为跑一个几何测试的门槛。
* **不做任何坐标系变换** —— 本模块只管像素。三维解算在 ``projector`` 里，
  坐标系变换在 ROS2 节点里用 tf2 做。三层分开，各自可测。
* **类别白名单可配** —— 侦查场景只关心人、车、以及随身物品，
  其余 COCO 类别默认过滤。
"""

from __future__ import annotations

import os

from dataclasses import dataclass, field, replace

import cv2
import numpy as np

# 侦查场景关心的 COCO 类别（名字，不是索引 —— 索引在不同权重版本间可能变）
DEFAULT_CLASSES = (
    "person",
    "bicycle", "car", "motorcycle", "bus", "truck",
    "backpack", "handbag", "suitcase",
)


@dataclass
class Detection2D:
    """一次二维检测的结果。

    纯数据，不含任何 ROS2 类型 —— 这样它能在没有 ROS2 的地方被构造和断言。
    """

    class_name: str
    class_id: int
    confidence: float
    bbox_xyxy: tuple[float, float, float, float]
    """像素坐标 ``(x1, y1, x2, y2)``。"""

    track_id: int | None = None
    """跟踪 ID（调用 ``track()`` 时才有）。用于去重。"""

    mask: np.ndarray | None = None
    """实例分割掩膜（bool，与原图同形状）。用 ``-seg`` 权重时才非空。"""

    keypoints: np.ndarray | None = None
    """``(17, 3)`` 的 COCO 关键点 ``(x, y, conf)``。用 ``-pose`` 权重时才非空。"""

    @property
    def width(self) -> float:
        x1, _, x2, _ = self.bbox_xyxy
        return abs(x2 - x1)

    @property
    def height(self) -> float:
        _, y1, _, y2 = self.bbox_xyxy
        return abs(y2 - y1)

    @property
    def aspect_ratio(self) -> float:
        """高 / 宽。倒地检测的方法 A 用它做粗筛。"""
        w = self.width
        return self.height / w if w > 1e-6 else 0.0


class DetectorUnavailableError(RuntimeError):
    """ultralytics 不可用。"""


def resolve_class_filter(names: dict, wanted: set[str], weights: str) -> list[int]:
    """把类别**名字**白名单解析成 ultralytics 要的**索引**列表。

    抽成独立函数同样是为了可测（它原先在 ``Detector.__init__`` 里，
    而那条路径需要加载 torch）。

    ⚠️ **一个都没匹配上时必须抛错**，不能返回空列表。

    ultralytics 的类别过滤是 ``filt = (x[:,5:6] == classes).any(1)``：
    传空列表时比较结果全为 False，于是**每帧返回空列表、永远**。
    上层看到的是「这个场景没有人」，而不是「配置错了」——
    一个静默的、看起来完全正常的零检出。

    注意空列表与 ``None`` 在 ultralytics 里是**两回事**：
    ``None`` = 不过滤（全要），``[]`` = 一个都不要。这里绝不能混同。
    """
    matched = [i for i, n in names.items() if n in wanted]
    missing = wanted - set(names.values())
    if missing:
        # 部分不匹配只报警 —— 权重换版本时类别名有出入是正常的。
        print(f"[Detector] 权重里没有这些类别，已忽略：{sorted(missing)}")

    if not matched:
        raise DetectorUnavailableError(
            f"类别白名单 {sorted(wanted)} 在权重 {weights!r} 里**一个都没匹配上**。\n"
            f"  该权重实际类别：{sorted(set(names.values()))[:15]}"
            f"{' ...' if len(names) > 15 else ''}\n"
            f"  继续跑下去会得到「每帧零检出」这种看起来正常的结果，所以这里直接报错。\n"
            f"  改 classes=... 或设 classes=None（不过滤）。"
        )
    return matched


def results_to_detections(result, names: dict, track: bool) -> list["Detection2D"]:
    """把一条 ultralytics 的 ``Results`` 映射成 ``Detection2D`` 列表。

    **抽成独立的纯函数是为了可测** —— 原先这段逻辑埋在 ``Detector._run`` 里，
    而 ``_run`` 需要真的加载 torch 才能跑，于是它成了整个模块里
    唯一 0 覆盖的部分（审查点名）。现在只要一个鸭子类型的假 result 就能测，
    不需要 torch、不需要权重文件。

    ``result`` 只需要满足：``boxes.xyxy/conf/cls``、可选的 ``boxes.id``、
    可选的 ``masks.data``、可选的 ``keypoints.data``，
    且各字段支持 ``.cpu().numpy()``（ultralytics 的 torch 张量就是这样）。
    """
    if result is None:
        return []
    boxes = getattr(result, "boxes", None)
    if boxes is None or len(boxes) == 0:
        return []

    xyxy = boxes.xyxy.cpu().numpy()
    confs = boxes.conf.cpu().numpy()
    clss = boxes.cls.cpu().numpy().astype(int)

    ids = None
    if track and getattr(boxes, "id", None) is not None:
        ids = boxes.id.cpu().numpy().astype(int)

    masks = None
    r_masks = getattr(result, "masks", None)
    if r_masks is not None and getattr(r_masks, "data", None) is not None:
        masks = r_masks.data.cpu().numpy()          # (N, H, W)，值域 0/1

    kpts = None
    r_kpts = getattr(result, "keypoints", None)
    if r_kpts is not None and getattr(r_kpts, "data", None) is not None:
        kpts = r_kpts.data.cpu().numpy()            # (N, 17, 3)

    out: list[Detection2D] = []
    for i in range(len(xyxy)):
        out.append(
            Detection2D(
                class_name=names.get(int(clss[i]), str(clss[i])),
                class_id=int(clss[i]),
                confidence=float(confs[i]),
                bbox_xyxy=tuple(float(v) for v in xyxy[i]),
                track_id=int(ids[i]) if ids is not None else None,
                mask=(masks[i] > 0.5) if masks is not None else None,
                keypoints=kpts[i] if kpts is not None else None,
            )
        )
    return out


def upscale_factor(h: int, w: int, min_side: int, max_factor: int = 4) -> int:
    """算「把短边抬到 ``min_side`` 以上」所需的**整数**放大倍数。

    整数是为了让还原是精确的除法（``/k``），也为了让掩膜能用整数步长抽回原尺寸。
    """
    if min_side <= 0:
        return 1
    short = min(h, w)
    if short <= 0 or short >= min_side:
        return 1
    return min(int(np.ceil(min_side / short)), max_factor)


def upscale_for_inference(image_bgr: np.ndarray, min_side: int,
                          max_factor: int = 4):
    """按需放大输入图。返回 ``(推理用图, 倍数)``；不需要放大时原样返回、倍数为 1。

    ---------------------------------------------------------------------------
    为什么要放大，以及为什么必须是 ``INTER_LANCZOS4``（2026-09-18 实测）

    在 ``1378_rgb_depth16.mkv``（Kinect v1，320x240）上，**人一躺下置信度就塌**：

        站立 #114  0.91      躺床 #342  **0.00**      躺地 #918  0.12

    类别判对了（``person``，不是误分成 ``dog``/``bed``），塌的是置信度，
    全部落到 0.25 门限以下 —— 于是关键帧被静默丢弃。
    这与跟踪器 ``new_track_thresh`` 那次**是同一个失效模式**，只是换了一层。

    固定算力预算（``imgsz=640``）下比不同预处理，4 个躺姿关键帧的均值置信度：

        不放大（ultralytics 内部 LINEAR 2x）   0.275
        LINEAR 2x + unsharp                    0.381
        LANCZOS4 2x -> 640x480                 **0.456**
        LANCZOS4 4x（又被缩回 640）             0.449

    四条结论，每条都排除了一个想当然的做法：

    * **起作用的是插值核，不是「放大」本身。** 内部那步放大用的是 ``INTER_LINEAR``
      —— 实测它与手工 ``INTER_LINEAR`` **逐帧完全相等**（0.238/0.179/0.301/0.382）。
      换成 ``LANCZOS4`` 才有提升。``INTER_AREA``（放大时是模糊）反而最差（0.00~0.10）。
    * **放大超过 2x 不再有收益**（0.449 vs 0.456）。所以只放大到够用为止。
    * **把 ``imgsz`` 开大不是替代方案**：``imgsz=960`` 时 ultralytics 内部用
      LINEAR 放 3x，结果反而**不如**手工 2x（0.091~0.230 vs 0.476~0.531）。
    * 附带否掉的：「LANCZOS4 + 反锐化掩膜」均值最高（0.474）但**跨帧方差大**
      （帧 576 从 0.364 掉到 0.145），不可靠，不采用。
    """
    if not isinstance(image_bgr, np.ndarray):
        # ultralytics 本身接受文件路径，但自动放大要读像素尺寸。
        # **不能**在这里「读不到就跳过放大」—— 那会让小分辨率输入静默退回
        # 置信度塌陷那条老路，正是这个机制要修的。宁可报错。
        raise TypeError(
            f"Detector 只接受 numpy 图像，收到 {type(image_bgr).__name__}。\n"
            f"  自动放大（upscale_min_side）需要先读像素尺寸；传路径的话这一步做不了，\n"
            f"  而跳过它会让低分辨率输入上的躺姿置信度静默塌到 conf 门限以下。\n"
            f"  用 cv2.imread(path) 读进来再传。"
        )

    h, w = image_bgr.shape[:2]
    k = upscale_factor(h, w, min_side, max_factor)
    if k == 1:
        return image_bgr, 1
    bigger = cv2.resize(image_bgr, (w * k, h * k), interpolation=cv2.INTER_LANCZOS4)
    return bigger, k


def rescale_detections(dets: list["Detection2D"], k: int,
                       out_hw: tuple[int, int]) -> list["Detection2D"]:
    """把「在放大 ``k`` 倍的图上」得到的检测**还原到原图坐标系**（``out_hw`` = 原图高宽）。

    ---------------------------------------------------------------------------
    ⚠️ 漏了这一步**不会报错**，这是它危险的地方

    关键点会整体偏到 2 倍远处。拿它去索引深度图时，采到的是**完全另一个位置**的深度，
    离地高度照常算得出来、照常是个合法浮点数 —— 只是全是错的。
    与「地面拟合锁错平面」属于同一类：不抛异常，结果全错。

    掩膜也必须一起还原：``retina_masks=True`` 让掩膜回到**输入图**分辨率，
    放大之后那个「输入图」就是放大图了。``Detection2D.mask`` 的契约是
    「与原图同形状」，所以这里用 ``INTER_NEAREST`` 抽回原尺寸（保持二值）。
    """
    if k == 1:
        return dets
    oh, ow = out_hw
    out = []
    for d in dets:
        bbox = tuple(v / k for v in d.bbox_xyxy)
        kps = None
        if d.keypoints is not None:
            kps = np.column_stack([d.keypoints[:, 0] / k,
                                   d.keypoints[:, 1] / k,
                                   d.keypoints[:, 2]])       # 置信度那一列不动
        mask = None
        if d.mask is not None:
            mask = cv2.resize(d.mask.astype(np.uint8), (ow, oh),
                              interpolation=cv2.INTER_NEAREST) > 0
        out.append(replace(d, bbox_xyxy=bbox, keypoints=kps, mask=mask))
    return out


def _load_ultralytics():
    """惰性导入。失败时给一条能直接照做的提示，而不是一个裸的 ImportError。"""
    try:
        from ultralytics import YOLO  # noqa: PLC0415
    except ImportError as exc:  # pragma: no cover - 取决于环境
        raise DetectorUnavailableError(
            "需要 ultralytics：pip install ultralytics。\n"
            "注意它依赖 torch（数 GB），装之前先确认磁盘余量。"
        ) from exc
    return YOLO


def _default_tracker_path() -> str:
    """本包自带的跟踪器配置。用绝对路径 —— ultralytics 会按 cwd 找相对路径。"""
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "config", "track_fall.yaml")


_DEFAULT_TRACKER = _default_tracker_path()


@dataclass
class DetectorConfig:
    """检测器配置。默认值都是**起点**，需按实测调整。"""

    weights: str = "yolo11n.pt"
    """权重文件。首次使用时 ultralytics 会自动下载到当前目录。"""

    conf: float = 0.25
    """置信度阈值。

    ⚠️ **与跟踪器有未文档化的耦合，调它之前先看这段**（2026-09-18）。

    检测框是**先过这个阈值、再进跟踪器**的（ultralytics 内部顺序如此），
    所以它同时是跟踪器能拿到的检测的**下界**：

    * 调到 > ``track_low_thresh``(0.10)：第二轮「用低分框把丢失目标找回来」
      就形同虚设 —— 那些框已经被这里滤掉了。
    * 调到 > ``new_track_thresh``(0.25)：**新建轨迹变得困难**。
      实测：人躺在地上时检测置信度只有 0.28~0.74，把门限抬到 0.6
      会让「进画面时就躺着的人」在建轨迹这一步就被挡住
      （20 帧里只有 8 帧拿得到 id，而不抬是 20/20）。
    * 而倒地判据完全建立在「同一个 id 的连续观测」上 —— 没有 id 就没有一切。

    **「提高 conf 降误检」是个很自然的动作，但它会静默地打断倒地链路。**
    真要降误检，改类别白名单或后处理，不要抬这个值。
    """

    iou: float = 0.5
    """NMS 的 IoU 阈值。"""

    imgsz: int = 640
    """推理分辨率。

    ⚠️ 这个值直接决定**能看多远**：640x480 输入下，20 m 外的人可能只有 20-30 px 高。
    定「检出率 > 85%」这类指标时，必须同时说明是在什么距离、什么 imgsz 下测的。
    """

    upscale_min_side: int = 480
    """输入短边小于它时，先用 ``INTER_LANCZOS4`` 整数倍放大到 ≥ 它再推理。``0`` = 关闭。

    ---------------------------------------------------------------------------
    实测依据见 :func:`upscale_for_inference` 的 docstring。一句话：
    **低分辨率输入上，躺姿的检测置信度会塌到门限以下**，
    而「躺下」正是本系统要检测的事件，所以这是致命的而不是精度损失。

    默认 **480** 的具体含义：

    * Kinect v1 的 ``320x240``（短边 240）→ 放大 **2x** 到 ``640x480``，
      正是实测最优的那一档；
    * RealSense 的 ``640x480``（短边 480）→ 倍数 1，**行为与改动前完全一致**；
    * 更大的输入也一律不动。

    也就是说这个默认值**只影响小分辨率输入**，不会悄悄改变现有 640x480 素材的结果。

    ⚠️ 放大倍数上限为 4（:func:`upscale_factor` 的 ``max_factor``），
    避免极小图被放大到吃掉大量显存。
    """

    classes: tuple[str, ...] | None = field(default_factory=lambda: DEFAULT_CLASSES)
    """关注的类别名白名单。``None`` 表示不过滤。"""

    device: str | None = None
    """``"cpu"`` / ``"0"`` / ``None``（自动）。"""

    tracker: str = _DEFAULT_TRACKER
    """跟踪器配置文件。**必须显式指定**，理由见下。

    ---------------------------------------------------------------------------
    ⚠️ 不要依赖 ultralytics 的隐式默认（2026-09-18，真实倒地录像实测）

    原先 ``Detector.track()`` 调 ``model.track(...)`` 时**不传** ``tracker=``，
    于是用 ultralytics 的 ``DEFAULT_CFG['tracker']``。在 8.4.154 上它是
    ``tracktrack.yaml``（既不是 bytetrack 也不是 botsort），实测在倒地视频上
    **id 会断**，倒地判据拿不到任何可用的轨迹。而隐式默认是**随版本变**的。

    默认值指向本包自带的 ``config/track_fall.yaml``，理由与消融实验见该文件。
    核心那一条：**``match_thresh`` 必须放宽到 0.95** ——
    它是关联代价上限，默认 0.8 要求 IoU>0.2，而人一倒地框从「高瘦」变
    「扁宽」，IoU 掉到 0.2 以下，旧轨迹就再也绑不回来，id 当场断掉。

    ⚠️ 换 ultralytics 版本或换素材后，用 ``tools/check_track_continuity.py``
    复核，**尤其是多人场景** —— 放宽关联的代价是两个人挨得近时更容易并成一条。
    """


class Detector:
    """YOLO 检测器的薄封装。

    用法::

        det = Detector(DetectorConfig(weights="yolo11n.pt"))
        for d in det.detect(image_bgr):
            print(d.class_name, d.confidence, d.bbox_xyxy)
    """

    def __init__(self, config: DetectorConfig | None = None):
        self.config = config or DetectorConfig()
        YOLO = _load_ultralytics()
        self._model = YOLO(self.config.weights)
        self._names = self._model.names  # {id: name}

        # 把类别名白名单解析成索引。放在这里做一次，而不是每帧做。
        if self.config.classes is None:
            self._class_filter = None
        else:
            self._class_filter = resolve_class_filter(
                self._names, set(self.config.classes), self.config.weights
            )

    # ------------------------------------------------------------------
    def detect(self, image_bgr: np.ndarray):
        """单帧检测，无跟踪。"""
        return self._run(image_bgr, track=False)

    def track(self, image_bgr: np.ndarray):
        """单帧检测 + 跟踪，带 ``track_id``。

        ``persist=True`` 让跟踪器在连续调用之间保持状态 —— 抽帧推理时这是必须的，
        否则每帧都被当成新序列，ID 会乱跳。
        """
        return self._run(image_bgr, track=True)

    # ------------------------------------------------------------------
    def _run(self, image_bgr: np.ndarray, track: bool) -> list[Detection2D]:
        # 小分辨率输入先放大 —— 否则躺姿置信度会塌到门限以下（见 upscale_min_side）。
        img_in, k = upscale_for_inference(image_bgr, self.config.upscale_min_side)

        kwargs = dict(
            conf=self.config.conf,
            iou=self.config.iou,
            imgsz=self.config.imgsz,
            verbose=False,
            # 让分割掩膜**回到原图分辨率**。
            # 默认（retina_masks=False）下掩膜是 letterbox 补边后的推理尺寸
            # （例如 640×640），而调用方拿原图坐标去切它 —— 会静默采错像素。
            # 这一条是对抗性审查指出的，虽然本仓库实测当前版本掩膜尺寸是对的，
            # 但那是行为不是契约，显式打开更稳。
            retina_masks=True,
        )
        if self.config.device is not None:
            kwargs["device"] = self.config.device
        if self._class_filter is not None:
            kwargs["classes"] = self._class_filter

        if track:
            # 显式指定跟踪器 —— 理由见 DetectorConfig.tracker 的说明：
            # 隐式默认是随 ultralytics 版本变的，而其中一种会让「躺在地上的人」
            # 完全拿不到 track_id。
            results = self._model.track(
                img_in, persist=True, tracker=self.config.tracker, **kwargs
            )
        else:
            results = self._model.predict(img_in, **kwargs)

        if not results:
            return []
        dets = results_to_detections(results[0], self._names, track=track)
        # 还原到原图坐标系 —— 漏掉不会报错，只会让所有坐标偏到 k 倍远处。
        return rescale_detections(dets, k, image_bgr.shape[:2])
