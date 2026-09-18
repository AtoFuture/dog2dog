"""倒地识别的时间判据：把「某一帧躯干是斜的」变成「这个人摔倒了」这个**事件**。

--------------------------------------------------------------------------------
为什么必须有这一层

``anomaly.assess_fall_from_keypoints_3d`` 判的是**单帧**的躯干朝向。
2026-09-18 在 57 帧真实数据上实测：它单独用**不成立** ——
正类倾角 [75, 86, 87, 88, 90, 94]，负类 [31, 37, 42, 51, 55, 60, 63, 88]，
**一个正常帧（88°）比最轻的倒地（75°）还高**，任何阈值都分不开。

最致命的一帧是人**四肢着地跪趴着**，测出 84° —— 内参、深度、关键点全都没错，
躯干确实是水平的，但它不是倒地。详见 ``docs/倒地判据实测-方法B.md``。

根因：**「躯干不水平」≠「人倒在地上了」**。
跪、蹲、弯腰捡东西、爬行，几何上与倒地几乎无法区分。

--------------------------------------------------------------------------------
这一层加的判据

1. **时间上下文** —— 摔倒是一个**过程**：人先站着，然后在很短时间内转成水平，
   并且**保持住**。跪/蹲/弯腰是短暂或可逆的；坐着不动的话，人也不会先站着再突然倒下。

   ⚠️ 但时间上下文**单独也不够**：跪着不动同样满足「从直立转为水平并保持」。

2. **离地高度** —— 这才是把跪和躺分开的量。跪着的人躯干离地约 0.5~0.7 m，
   躺下的人约 0.2 m。同一个地面平面（``anomaly`` 那条链路上已经拟合出来了）就能算。

   ⚠️ **但本仓库目前的深度质量还标定不了这个阈值**，见下面 §高度门的现状。
   所以它默认**关闭**，只作为观测量上报。

3. **轨迹连续性** —— 这一切建立在「同一个 ``id`` 的多帧观测」上。
   所以倒地类**不允许** ``id=0``（那是跟踪器未确认轨迹时的保留值）。

--------------------------------------------------------------------------------
§高度门的现状（2026-09-18 实测，诚实记录）

在出厂的 57 帧上量了「躯干代表点离地高度」：

    真倒地（label 2/3）髋部离地  0.28 ~ 1.58 m
    正常活动（label 0）  髋部离地  0.22 ~ 1.51 m

**完全重叠，而且真倒地那侧出现了 1.58 m 这种不可能的值** ——
一个躺在地上的人髋部不可能离地 1.58 m。这是深度测量错误，不是判据问题。

所以 ``max_height_m`` 默认 **None（关闭）**：宁可不开，
也不要开一个没标定过的门 —— 那只会把真倒地也误杀掉。
字段照常上报，等深度质量修好、或者拿到更好的标定之后再打开。

"""

from __future__ import annotations

import enum
from dataclasses import dataclass, field

# ----------------------------------------------------------------------
# 判据参数
# ----------------------------------------------------------------------
UPRIGHT_MAX_DEG = 30.0
"""躯干与竖直方向夹角小于此值算「直立」。"""

HORIZONTAL_MIN_DEG = 60.0
"""大于此值算「已躺平」。

和 ``UPRIGHT_MAX_DEG`` 之间留出 30° 的空档是**迟滞** ——
没有迟滞的话，一个人躺在 59°/61° 附近抖动会反复重置状态。
空档内不改变当前状态。
"""

TRANSITION_MAX_S = 3.0
"""从「最后见到直立」到「转为水平」允许的最长时间。

超过它就说明不是摔倒 —— 人是慢慢躺下的（自己躺下休息、做康复训练）。
真人摔倒的躯干翻转在 1 秒量级，3 秒已经很宽松了。
"""

PERSIST_S = 2.0
"""水平状态要**保持**这么久才算确认。

这一条挡的是「弯腰捡东西后立刻直起来」和「蹲下又站起来」——
它们的水平段通常不到 1 秒。
"""

REFIRE_COOLDOWN_S = 10.0
"""同一轨迹要**连续直立**这么久，才允许再次上报倒地。

⚠️ 这个阈值量的是**直立时长**，不是「两次上报的间隔」——
后一种写法有个隐蔽的 bug：一个人倒地后一直躺着，间隔迟早会超过冷却，
于是每隔 10 秒被上报一次「摔倒了」。实测中就是这样：
躺着 20 秒报了 2 次。

正确的语义是「他起来过，所以可能又摔了一次」。
所以短暂抬头/翻身（直立不到这么久）不重置，真站起来才算。
"""


class TorsoState(enum.Enum):
    UNKNOWN = "UNKNOWN"
    UPRIGHT = "UPRIGHT"
    LYING = "LYING"


@dataclass
class TorsoObservation:
    """某个轨迹在某一帧的躯干观测。"""

    stamp: float
    """时刻（秒）。**同一轨迹内必须单调不减** —— 状态机靠时间差判断。"""

    tilt_deg: float | None
    """三维躯干与重力方向的夹角（度）。``None`` = 这一帧没测出来。"""

    height_m: float | None = None
    """躯干代表点离地高度（米）。``None`` = 没测出来。"""


@dataclass
class FallEvent:
    """一次确认的倒地事件。"""

    onset_stamp: float
    """**事件起始时刻** —— 首次观测到水平的时刻，不是确认时刻。

    上报出去的是这个值：评测关心的是「什么时候摔的」，
    而确认时刻取决于 ``PERSIST_S`` 这个内部参数，不该泄漏到接口上。
    """

    track_id: int
    tilt_deg: float
    height_m: float | None
    detected_at: float


@dataclass
class _Track:
    state: TorsoState = TorsoState.UNKNOWN
    last_upright_at: float | None = None
    upright_since: float | None = None
    lying_since: float | None = None
    fired_at: float | None = None
    last_stamp: float | None = None


@dataclass
class FallTracker:
    """按轨迹维护时间状态机，输出倒地**事件**。

    用法::

        tracker = FallTracker()
        for obs in stream:
            ev = tracker.update(track_id, obs)
            if ev is not None:
                publish_fall(ev)          # 每条轨迹每个事件只发一次

    ⚠️ **首次见到时人已经躺着的轨迹不会产生事件** ——
    没有「直立」这个前状态，就无从判断他是摔倒的还是本来就在那儿躺着。
    这是判据的固有边界，不是 bug：真机上应当由「目标首次进入视野时给一个
    更长的观察期」来缓解，而不是猜。
    """

    upright_max_deg: float = UPRIGHT_MAX_DEG
    horizontal_min_deg: float = HORIZONTAL_MIN_DEG
    transition_max_s: float = TRANSITION_MAX_S
    persist_s: float = PERSIST_S
    refire_cooldown_s: float = REFIRE_COOLDOWN_S

    max_height_m: float | None = None
    """离地高度上限。``None`` = **不启用高度门**（当前默认，理由见模块 docstring）。"""

    _tracks: dict = field(default_factory=dict)

    # ------------------------------------------------------------------
    def update(self, track_id: int, obs: TorsoObservation) -> FallEvent | None:
        """喂一帧观测，若确认倒地则返回事件，否则 ``None``。"""
        tr = self._tracks.setdefault(track_id, _Track())

        if tr.last_stamp is not None and obs.stamp < tr.last_stamp:
            raise ValueError(
                f"轨迹 {track_id} 的时间戳回退了（{tr.last_stamp} -> {obs.stamp}）；"
                "状态机靠时间差判断，回退的时间戳会给出没有意义的结果"
            )
        tr.last_stamp = obs.stamp

        tilt = obs.tilt_deg
        if tilt is None:
            # 这一帧没有可用观测。**不推进任何状态** ——
            # 把「没测出来」当成「不水平」会让漏检悄悄变成「没摔倒」
            return None

        if tilt <= self.upright_max_deg:
            if tr.state is not TorsoState.UPRIGHT:
                tr.upright_since = obs.stamp
            elif tr.upright_since is not None and \
                    obs.stamp - tr.upright_since >= self.refire_cooldown_s:
                # 连续直立够久 = 他真的起来过，所以「又摔一次」是可能的。
                # 短暂抬头/翻身到不了这里，于是不会重复上报。
                tr.fired_at = None
            tr.state = TorsoState.UPRIGHT
            tr.last_upright_at = obs.stamp
            tr.lying_since = None
            return None

        if tilt >= self.horizontal_min_deg:
            if tr.state is not TorsoState.LYING:
                tr.state = TorsoState.LYING
                tr.lying_since = obs.stamp

            if tr.fired_at is not None:
                return None
            if tr.last_upright_at is None:
                return None                       # 没见它直立过 -> 判不了
            if tr.lying_since - tr.last_upright_at > self.transition_max_s:
                return None                       # 太慢 -> 是躺下，不是摔倒
            if obs.stamp - tr.lying_since < self.persist_s:
                return None                       # 还没保持够久
            if self.max_height_m is not None and obs.height_m is not None \
                    and obs.height_m > self.max_height_m:
                return None                       # 离地太高 -> 是跪着/蹲着

            tr.fired_at = obs.stamp
            return FallEvent(
                onset_stamp=tr.lying_since,
                track_id=track_id,
                tilt_deg=tilt,
                height_m=obs.height_m,
                detected_at=obs.stamp,
            )

        # 迟滞区（upright_max < tilt < horizontal_min）：保持当前状态不变
        return None

    # ------------------------------------------------------------------
    def forget(self, track_id: int) -> None:
        """轨迹消失时清掉状态。"""
        self._tracks.pop(track_id, None)

    def prune(self, now: float, max_age_s: float = 30.0) -> int:
        """清掉 ``max_age_s`` 没再出现的轨迹，返回清掉几条。

        **不清理的后果**：跟踪器在目标离开画面后会分配新 id，
        旧 id 的状态永远留在字典里 —— 一次长实验下来是个只涨不跌的内存泄漏。

        用「多久没出现」而不是「跟踪器说它还在不在」：后者需要节点
        额外维护一套 id 生命周期，而这里只需要一个保守的超时。
        """
        stale = [tid for tid, tr in self._tracks.items()
                 if tr.last_stamp is not None and now - tr.last_stamp > max_age_s]
        for tid in stale:
            del self._tracks[tid]
        return len(stale)

    @property
    def n_tracks(self) -> int:
        return len(self._tracks)
