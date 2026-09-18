"""``detector`` 模块的测试。

⚠️ 这个文件补的是审查点名的一个盲区：原先 **``detector.py`` 整模块 0% 覆盖** ——
`Detector` 类没有任何测试 import 过它，`_run()` 里从 ultralytics 输出到
`Detection2D` 的全部映射逻辑（类别名解析、track/无 track、掩膜、关键点）
从未被执行。

之所以能在这里测而不需要 torch：映射逻辑已经抽成了
``results_to_detections`` / ``resolve_class_filter`` 两个**纯函数**，
只要一个鸭子类型的假 result 就能驱动。
"""

from __future__ import annotations

import cv2
import numpy as np
import pytest

from g2_core.detector import (
    DEFAULT_CLASSES,
    Detection2D,
    DetectorConfig,
    DetectorUnavailableError,
    resolve_class_filter,
    results_to_detections,
)

# ----------------------------------------------------------------------
# 假的 ultralytics 输出
# ----------------------------------------------------------------------
COCO_NAMES = {0: "person", 1: "bicycle", 2: "car", 3: "motorcycle", 7: "truck"}


class _FakeTensor:
    """模仿 torch 张量：只要支持 .cpu().numpy() 就够了。"""

    def __init__(self, arr):
        self._a = np.asarray(arr)

    def cpu(self):
        return self

    def numpy(self):
        return self._a


class _FakeBoxes:
    def __init__(self, xyxy, conf, cls, ids=None):
        self.xyxy = _FakeTensor(xyxy)
        self.conf = _FakeTensor(conf)
        self.cls = _FakeTensor(cls)
        self.id = _FakeTensor(ids) if ids is not None else None

    def __len__(self):
        return len(self.xyxy._a)


class _FakeResult:
    def __init__(self, boxes=None, masks=None, keypoints=None):
        self.boxes = boxes
        self.masks = masks
        self.keypoints = keypoints


class _FakeMasks:
    def __init__(self, data):
        self.data = _FakeTensor(data)


class _FakeKeypoints:
    def __init__(self, data):
        self.data = _FakeTensor(data)


def _boxes(n=1):
    return _FakeBoxes(
        xyxy=[[10.0, 20.0, 30.0, 60.0]] * n,
        conf=[0.9] * n,
        cls=[0] * n,
    )


# ----------------------------------------------------------------------
# 类别白名单解析
# ----------------------------------------------------------------------
def test_class_filter_resolves_names_to_indices():
    got = resolve_class_filter(COCO_NAMES, {"person", "car"}, "w.pt")
    assert set(got) == {0, 2}


def test_class_filter_partial_match_only_warns():
    """部分不匹配只报警（权重换版本时类别名有出入是正常的），不报错。"""
    got = resolve_class_filter(COCO_NAMES, {"person", "traffic_light"}, "w.pt")
    assert got == [0], "匹配上的仍要保留"


def test_class_filter_all_missing_raises():
    """🔴 一个都没匹配上必须抛错。

    这是审查发现的一个**静默失效**：原先只 print 一行警告，
    然后把空列表传给 ultralytics。而 ultralytics 的过滤是
    `(cls == classes).any(1)` —— 空列表会让比较结果全为 False，
    于是**每帧返回空列表、永远**。

    上层看到的现象是「这个场景里没有人」，而不是「配置错了」。
    一个看起来完全正常的零检出，比崩溃危险得多。
    """
    with pytest.raises(DetectorUnavailableError, match="一个都没匹配上"):
        resolve_class_filter({"0": "fire", "1": "smoke"}, {"person"}, "custom.pt")


def test_class_filter_error_message_lists_actual_classes():
    """报错信息要能直接照着改 —— 得告诉用户权重里到底有什么。"""
    with pytest.raises(DetectorUnavailableError) as ei:
        resolve_class_filter({"0": "fire", "1": "smoke"}, {"person"}, "custom.pt")
    msg = str(ei.value)
    assert "fire" in msg and "smoke" in msg


def test_default_class_list_matches_coco():
    """默认白名单里的名字必须都是 COCO 类别 —— 防止手滑写错一个词。"""
    coco = {
        "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
        "truck", "boat", "backpack", "umbrella", "handbag", "suitcase",
    }
    assert set(DEFAULT_CLASSES) <= coco


# ----------------------------------------------------------------------
# 结果映射
# ----------------------------------------------------------------------
def test_none_and_empty_results():
    assert results_to_detections(None, COCO_NAMES, track=False) == []
    assert results_to_detections(_FakeResult(), COCO_NAMES, track=False) == []
    assert results_to_detections(
        _FakeResult(boxes=_FakeBoxes([], [], [])), COCO_NAMES, track=False
    ) == []


def test_basic_mapping():
    r = _FakeResult(boxes=_FakeBoxes([[10.0, 20.0, 30.0, 60.0]], [0.87], [0]))
    dets = results_to_detections(r, COCO_NAMES, track=False)

    assert len(dets) == 1
    d = dets[0]
    assert d.class_name == "person"
    assert d.class_id == 0
    assert d.confidence == pytest.approx(0.87)
    assert d.bbox_xyxy == (10.0, 20.0, 30.0, 60.0)
    assert d.track_id is None, "不跟踪时不该有 id"


def test_unknown_class_id_falls_back_to_string():
    """权重里没有的类别 id 不能崩，退化成字符串即可。"""
    r = _FakeResult(boxes=_FakeBoxes([[0, 0, 1, 1]], [0.5], [99]))
    d = results_to_detections(r, COCO_NAMES, track=False)[0]
    assert d.class_name == "99"


def test_track_id_only_when_tracking():
    boxes = _FakeBoxes([[0, 0, 10, 10]], [0.9], [0], ids=[7])

    assert results_to_detections(_FakeResult(boxes=boxes), COCO_NAMES, track=False)[0].track_id is None
    assert results_to_detections(_FakeResult(boxes=boxes), COCO_NAMES, track=True)[0].track_id == 7


def test_mask_is_thresholded_to_bool():
    """掩膜要二值化成 bool —— 下游用它做逐像素采样，浮点概率值没有意义。"""
    mask = np.array([[[0.1, 0.9], [0.6, 0.4]]], dtype=np.float32)
    r = _FakeResult(boxes=_boxes(), masks=_FakeMasks(mask))

    d = results_to_detections(r, COCO_NAMES, track=False)[0]
    assert d.mask is not None
    assert d.mask.dtype == bool
    assert d.mask.tolist() == [[False, True], [True, False]]


def test_keypoints_pass_through():
    kpts = np.zeros((1, 17, 3), dtype=np.float32)
    kpts[0, 5] = [10.0, 20.0, 0.9]
    r = _FakeResult(boxes=_boxes(), keypoints=_FakeKeypoints(kpts))

    d = results_to_detections(r, COCO_NAMES, track=False)[0]
    assert d.keypoints is not None
    assert d.keypoints.shape == (17, 3)
    assert d.keypoints[5].tolist() == pytest.approx([10.0, 20.0, 0.9])


def test_multiple_boxes_keep_order():
    r = _FakeResult(boxes=_FakeBoxes(
        [[0, 0, 1, 1], [2, 2, 3, 3], [4, 4, 5, 5]], [0.9, 0.8, 0.7], [0, 2, 7]
    ))
    dets = results_to_detections(r, COCO_NAMES, track=False)
    assert [d.class_name for d in dets] == ["person", "car", "truck"]
    assert [d.confidence for d in dets] == pytest.approx([0.9, 0.8, 0.7])


# ----------------------------------------------------------------------
# Detection2D 自身的派生属性（方法A 的入口，此前也从未被执行）
# ----------------------------------------------------------------------
def test_detection2d_derived_properties():
    d = Detection2D("person", 0, 0.9, (10.0, 20.0, 40.0, 80.0))
    assert d.width == pytest.approx(30.0)
    assert d.height == pytest.approx(60.0)
    assert d.aspect_ratio == pytest.approx(2.0)


def test_aspect_ratio_of_degenerate_box():
    """零宽框不能让 aspect_ratio 除零崩掉。"""
    d = Detection2D("person", 0, 0.9, (5.0, 5.0, 5.0, 50.0))
    assert d.aspect_ratio == 0.0


def test_track_passes_an_explicit_tracker(monkeypatch):
    """必须显式传跟踪器，不能靠 ultralytics 的隐式默认。

    实测（fall-01-cam0.mp4，真实倒地录像）：隐式默认是 ``tracktrack.yaml``，
    ``new_track_thresh=0.7``，而人躺在地上时检测置信度只有 0.28~0.74 ——
    轨迹根本建不起来，**落地后 0/31 帧有 id**。换成 bytetrack 是 24/31。

    没有 id，倒地判据（fall_tracker）就完全无从下手。
    隐式默认还会随 ultralytics 版本变，所以这里钉死「必须传」。
    """
    from g2_core.detector import Detector, DetectorConfig

    captured = {}

    class _FakeModel:
        def track(self, img, **kw):
            captured.update(kw)
            return []

    d = Detector.__new__(Detector)
    d.config = DetectorConfig()
    d._model = _FakeModel()
    d._class_filter = None

    d.track(np.zeros((8, 8, 3), dtype=np.uint8))

    assert "tracker" in captured, "没传 tracker 就会用随版本变的隐式默认"
    # 具体是哪个文件不重要，重要的是**必须是本包自带的那份** ——
    # 它的 match_thresh 放宽过（默认 bytetrack.yaml 的 0.8 会让 id 在倒地瞬间断掉）
    assert captured["tracker"].endswith("track_fall.yaml")
    assert captured["persist"] is True


def test_track_fall_yaml_only_changes_match_thresh():
    """回归：``config/track_fall.yaml`` 与内置 ``bytetrack.yaml`` 只许差一个参数。

    ⚠️ 这条测试的由来（2026-09-18）：本文件初版把 ``track_high_thresh`` /
    ``track_low_thresh`` / ``new_track_thresh`` 写成了 0.6 / 0.25 / 0.6，
    并标注「保持默认」—— **而 bytetrack 的真实默认是 0.25 / 0.10 / 0.25**，
    写进去的那三个值其实是 ``tracktrack.yaml``（就是要避开的那份）的。
    后果实测：人进画面时**已经躺着**的话，20 帧里只有 8 帧拿得到 id
    （用真默认是 20/20）—— 正是那份配置想修的失效模式，换了个地方又犯一遍。

    这类错误**没有任何报错**，只会表现为「倒地事件永远是 0」。
    所以钉一条测试：想改别的参数，就得先想清楚并改这里。
    """
    import os
    import yaml

    import ultralytics

    builtin = os.path.join(os.path.dirname(ultralytics.__file__),
                           "cfg", "trackers", "bytetrack.yaml")
    ours = DetectorConfig().tracker

    theirs = yaml.safe_load(open(builtin))
    mine = yaml.safe_load(open(ours))

    diff = {k: (theirs.get(k), mine.get(k)) for k in set(theirs) | set(mine)
            if theirs.get(k) != mine.get(k)}

    assert set(diff) == {"match_thresh"}, (
        f"track_fall.yaml 与内置 bytetrack.yaml 的差异不止 match_thresh：{diff}\n"
        f"（改别的参数前请先确认那不是「凭印象写的默认值」）"
    )
    assert mine["match_thresh"] == 0.95


# ----------------------------------------------------------------------
# 小分辨率放大（2026-09-18）
# ----------------------------------------------------------------------
def test_upscale_factor_semantics():
    """只有短边低于目标时才放大，且倍数是整数。"""
    from g2_core.detector import upscale_factor

    assert upscale_factor(240, 320, 480) == 2          # Kinect 320x240 → 2x
    assert upscale_factor(480, 640, 480) == 1          # RealSense 不动
    assert upscale_factor(720, 1280, 480) == 1         # 更大也不动
    assert upscale_factor(240, 320, 0) == 1            # 0 = 关闭
    assert upscale_factor(200, 200, 480) == 3          # ceil(480/200)=3
    assert upscale_factor(479, 479, 480) == 2          # 差一点点也要抬上去


def test_upscale_factor_is_capped():
    """极小图不许被放大到天文数字（会吃掉大量显存）。"""
    from g2_core.detector import upscale_factor

    assert upscale_factor(10, 10, 480) == 4
    assert upscale_factor(10, 10, 480, max_factor=2) == 2


def test_upscale_for_inference_noop_returns_same_object():
    """倍数 1 时必须原样返回 —— 不要白白复制一帧图像。"""
    from g2_core.detector import upscale_for_inference

    img = np.zeros((480, 640, 3), dtype=np.uint8)
    out, k = upscale_for_inference(img, min_side=480)
    assert k == 1
    assert out is img


def test_upscale_for_inference_rejects_non_array_input():
    """传路径要**明确报错**，不能静默跳过放大。

    ultralytics 自己接受文件路径，所以「传路径」是个很自然的用法；
    但读不到像素尺寸就没法放大，而静默跳过等于悄悄退回置信度塌陷那条老路。
    """
    from g2_core.detector import upscale_for_inference

    with pytest.raises(TypeError, match="numpy 图像"):
        upscale_for_inference("some/image.jpg", min_side=480)


def test_upscale_for_inference_uses_lanczos4():
    """钉死插值核是 ``INTER_LANCZOS4``。

    这不是吹毛求疵 —— 实测（1378 素材，4 个躺姿关键帧均值置信度）：
    ``INTER_LINEAR`` 0.275 / ``INTER_AREA`` 0.05 / ``INTER_CUBIC`` 0.42 /
    ``INTER_LANCZOS4`` **0.456**。换成别的核，躺姿就又开始往门限下面掉，
    而**不会有任何报错**。
    """
    from g2_core.detector import upscale_for_inference

    # 棋盘格：不同插值核的结果差异最大，最容易把换核钉出来
    img = np.indices((240, 320)).sum(axis=0) % 2
    img = np.repeat((img * 255).astype(np.uint8)[:, :, None], 3, axis=2)

    out, k = upscale_for_inference(img, min_side=480)
    expected = cv2.resize(img, (640, 480), interpolation=cv2.INTER_LANCZOS4)

    assert k == 2
    assert out.shape == (480, 640, 3)
    assert np.array_equal(out, expected)


def test_rescale_detections_restores_original_coordinates():
    """放大后必须把坐标除回去 —— 漏掉不会报错，只会让所有坐标偏到 k 倍远处。"""
    from g2_core.detector import Detection2D, rescale_detections

    kps = np.zeros((17, 3), dtype=float)
    kps[0] = [20.0, 40.0, 0.9]
    kps[1] = [60.0, 120.0, 0.4]
    d = Detection2D("person", 0, 0.9, (20.0, 40.0, 60.0, 120.0), keypoints=kps)

    out = rescale_detections([d], k=2, out_hw=(240, 320))

    assert out[0].bbox_xyxy == (10.0, 20.0, 30.0, 60.0)
    assert out[0].keypoints[0].tolist() == [10.0, 20.0, 0.9]
    assert out[0].keypoints[1].tolist() == [30.0, 60.0, 0.4]   # 置信度列不动
    assert out[0].confidence == 0.9


def test_rescale_detections_resizes_mask_back():
    """``mask`` 的契约是「与原图同形状」，放大之后必须还原。"""
    from g2_core.detector import Detection2D, rescale_detections

    mask = np.zeros((480, 640), dtype=bool)
    mask[100:200, 200:300] = True
    d = Detection2D("person", 0, 0.9, (10.0, 20.0, 30.0, 60.0), mask=mask)

    out = rescale_detections([d], k=2, out_hw=(240, 320))

    assert out[0].mask.shape == (240, 320)
    assert out[0].mask.dtype == bool
    assert out[0].mask.any()


def test_rescale_detections_noop_at_factor_one():
    from g2_core.detector import Detection2D, rescale_detections

    d = Detection2D("person", 0, 0.9, (10.0, 20.0, 30.0, 60.0))
    assert rescale_detections([d], k=1, out_hw=(240, 320)) == [d]


def test_run_feeds_upscaled_image_and_returns_original_coordinates():
    """端到端钉死：模型**看到的是放大图**，调用方**拿到的是原图坐标**。

    这两件事必须同时成立。只做前者会让所有坐标偏到 2 倍远处（不报错），
    只做后者等于没放大（躺姿置信度照塌）。
    """
    from g2_core.detector import Detector, DetectorConfig

    seen = {}

    class _Model:
        def predict(self, img, **kw):
            seen["img"] = img
            kps = np.zeros((17, 3), dtype=float)
            kps[0] = [20.0, 40.0, 0.9]        # 放大图坐标系
            return [_FakeResult(
                boxes=_FakeBoxes(xyxy=[[20.0, 40.0, 60.0, 120.0]],
                                 conf=[0.9], cls=[0]),
                keypoints=_FakeKeypoints([kps]),
            )]

    d = Detector.__new__(Detector)
    d.config = DetectorConfig(upscale_min_side=480)
    d._model = _Model()
    d._names = COCO_NAMES
    d._class_filter = None

    out = d.detect(np.zeros((240, 320, 3), dtype=np.uint8))

    assert seen["img"].shape == (480, 640, 3), "模型没拿到放大图"
    assert out[0].bbox_xyxy == (10.0, 20.0, 30.0, 60.0), "坐标没还原"
    assert out[0].keypoints[0].tolist() == [10.0, 20.0, 0.9]


def test_default_config_upscales_small_input():
    """⚠️ 回归：**默认配置**就必须放大 320x240 输入。

    这条测试的由来（2026-09-18）：变异测试把 ``upscale_min_side`` 的默认值
    从 480 改成 0（等于关掉整个机制），**全部测试照过** ——
    因为其它用例要么显式传了 480，要么用的是 640x480 输入（本来就是倍数 1）。
    于是「默认值被改坏」这件事没有任何东西守着。

    这与 ``track_fall.yaml`` 那次是同一类：**默认值悄悄失守，行为静默退化**。
    后果具体是什么：Kinect 320x240 下躺姿置信度从 0.53 掉回 0.00，
    落到 conf 门限以下，倒地事件永远是 0，没有任何报错。
    """
    from g2_core.detector import Detector, DetectorConfig

    seen = {}

    class _Model:
        def predict(self, img, **kw):
            seen["img"] = img
            return []

    d = Detector.__new__(Detector)
    d.config = DetectorConfig()                 # ← 默认，不传任何东西
    d._model = _Model()
    d._names = COCO_NAMES
    d._class_filter = None

    d.detect(np.zeros((240, 320, 3), dtype=np.uint8))

    assert seen["img"].shape == (480, 640, 3), (
        f"默认配置没有放大 320x240 输入（模型拿到 {seen['img'].shape}）—— "
        f"躺姿置信度会塌回门限以下"
    )


def test_run_without_upscale_keeps_original_behaviour():
    """640x480 输入下倍数必须是 1 —— 默认值不许悄悄改动现有素材的结果。"""
    from g2_core.detector import Detector, DetectorConfig

    seen = {}

    class _Model:
        def predict(self, img, **kw):
            seen["img"] = img
            return [_FakeResult(boxes=_boxes())]

    d = Detector.__new__(Detector)
    d.config = DetectorConfig()          # 默认 upscale_min_side=480
    d._model = _Model()
    d._names = COCO_NAMES
    d._class_filter = None

    out = d.detect(np.zeros((480, 640, 3), dtype=np.uint8))

    assert seen["img"].shape == (480, 640, 3)
    assert out[0].bbox_xyxy == (10.0, 20.0, 30.0, 60.0)
