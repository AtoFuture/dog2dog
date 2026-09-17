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

import numpy as np
import pytest

from g2_core.detector import (
    DEFAULT_CLASSES,
    Detection2D,
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
