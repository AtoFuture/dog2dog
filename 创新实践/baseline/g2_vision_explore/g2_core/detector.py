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

from dataclasses import dataclass, field

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


@dataclass
class DetectorConfig:
    """检测器配置。默认值都是**起点**，需按实测调整。"""

    weights: str = "yolo11n.pt"
    """权重文件。首次使用时 ultralytics 会自动下载到当前目录。"""

    conf: float = 0.25
    """置信度阈值。"""

    iou: float = 0.5
    """NMS 的 IoU 阈值。"""

    imgsz: int = 640
    """推理分辨率。

    ⚠️ 这个值直接决定**能看多远**：640x480 输入下，20 m 外的人可能只有 20-30 px 高。
    定「检出率 > 85%」这类指标时，必须同时说明是在什么距离、什么 imgsz 下测的。
    """

    classes: tuple[str, ...] | None = field(default_factory=lambda: DEFAULT_CLASSES)
    """关注的类别名白名单。``None`` 表示不过滤。"""

    device: str | None = None
    """``"cpu"`` / ``"0"`` / ``None``（自动）。"""


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
            results = self._model.track(image_bgr, persist=True, **kwargs)
        else:
            results = self._model.predict(image_bgr, **kwargs)

        if not results:
            return []
        return results_to_detections(results[0], self._names, track=track)
