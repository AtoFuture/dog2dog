# 议题：探索状态机卡死在 `SENDING`

> 提出人：G2 ｜ 日期：2026-09-18 ｜ 状态：**待拍板**
> 来源：`g2_vision_explore/docs/对抗性审核-2026-09-18.md` P0-①

---

## 一、问题一句话

**`SENDING` 是一个没有出口的陷阱态：`select()` 失败时，文档给的两条收尾路径都会变成空操作，
状态机永久停在那里，而且不记任何异常。**

```python
# state_machine.py · next_command()
if self.phase is Phase.IDLE:
    self.phase = Phase.SENDING     # ★ 先置位
    return Command.SEND_GOAL
```

节点拿到 `SEND_GOAL` **之后**才去跑 `select()`。所以 `select()` 失败时，
状态机已经站在 `SENDING` 上了 —— 而这个状态的所有出口都假定「有一个目标正在发出」。

---

## 二、转移图

```
                    next_command()
      IDLE ─────────────────────────▶ SENDING ──── on_goal_sent() ────▶ NAVIGATING
                                          │           （select 成功）        ✓ 正常
                                          │
                                      select 失败
                                          │
                                          ├──✗ on_nav_timeout()   守卫要求 NAVIGATING
                                          └──✗ on_exhausted()     守卫要求 IDLE/PASSIVE
                                                    ↓
                                            没有出口 · phase 永久停在 SENDING
                                            next_command() 之后一律 NONE · anomalies 为空
```

**实测**（三条分支都跑过）：

```
select()=NO_CANDIDATE → on_nav_timeout() → False → phase=SENDING → 后续 5 拍全 NONE
select()=NO_FRONTIER  → on_exhausted()   →       → phase=SENDING → 后续 5 拍全 NONE
对照组（先 on_goal_sent() 再超时）              → NAVIGATING → 超时 → IDLE   ✓ 正常
```

---

## 三、三处说法互相矛盾

| 来源 | 说法 |
|---|---|
| **文档** | 类 docstring 的用法示例（就在节点该照抄的那段里）写：「`on_nav_timeout()` —— 暂时选不出点，**回 IDLE 稍后再试**」 |
| **测试** | `test_nav_timeout_works_from_sending` —— 名字说「能从 SENDING 救出来」，**函数体断言的却是救不出来**（`assert sm.phase is Phase.SENDING`），然后改调 `send_failed()` |
| **实现** | 两条守卫都放行不了 `SENDING`，**永久卡死且不记 anomaly**。唯一出口是 `on_goal_sent()` / `send_failed()`，而 `select()` 失败时两者都不会被调用 |

**现在没人知道哪个是对的行为。** 下一个人改这里时会照着自己读到的那份改。

---

## 四、为什么不是小事

1. **`NO_CANDIDATE` 不罕见** —— 在 400 张随机栅格图上跑了 **256 次**。
   任意一次发生在第一拍，G2 就**一次目标都发不出去**，而且是静默的。
   （触发场景包括：开机时地图/位姿还没就绪、候选点被黑名单滤光、安全半径不过。）
2. **`NO_FRONTIER` 分支吞掉「探索完成」信号** —— 永远到不了 `DONE`，
   「探索覆盖率」这条指标直接失真。
3. **这是 `SelectStatus` 当初想根治的那类静默失败换了张脸** ——
   修复把问题从「`None` → DONE」搬成了「`SENDING` → 无出口」。

---

## 五、三条候选路线

### 甲 · 放宽两条守卫 ｜ 2 行

```python
def on_nav_timeout(self) -> bool:
    if self.phase not in (Phase.NAVIGATING, Phase.SENDING):
        return False

def on_exhausted(self) -> None:
    if self.phase in (Phase.IDLE, Phase.PASSIVE, Phase.SENDING):
        self.phase = Phase.DONE
```

| | |
|---|---|
| **语义** | `SENDING` = 「一条 SEND_GOAL 指令已发出、还没落地」。`select()` 失败就是这条指令没落地，允许任一收尾路径关掉它 |
| **优点** | 改动最小，**节点侧一行不用动**，现有调用方式立刻能工作 |
| **缺点** | 「名字与时机不符」的问题留着 —— 在一个什么都没在导航的时刻调 `on_nav_timeout()`。若将来真出现「发出途中」的调用，会重复发目标 |

### 乙 · 加一个显式事件 ｜ 状态机 +6 行 / 节点 +1 分支 ⭐

```python
def on_select_failed(self, exhausted: bool = False) -> bool:
    """SEND_GOAL 已给出，但没有目标要发。"""
    if self.phase is not Phase.SENDING:
        return False
    self.phase = Phase.DONE if exhausted else Phase.IDLE
    self._cancel_issued = False
    return True

# 节点侧
if result.status is SelectStatus.GOAL:
    send_goal(result.goal); sm.on_goal_sent()
else:
    sm.on_select_failed(result.status is SelectStatus.NO_FRONTIER)
```

| | |
|---|---|
| **语义** | 把「指令已发出」的三种结局写全：`on_goal_sent()` 落地、`send_failed()` 发不出去、`on_select_failed()` 没得发。三者互斥且完备 |
| **优点** | **唯一把根因真正解掉的方案** —— 根因是 `SENDING` 混淆了「正在发出」和「刚决定要选点」两种情形 |
| **缺点** | 新增一个方法；要写清楚它和 `send_failed()` 的区别（「发不出去」vs「没得发」），否则下一个人会混用 |

### 丙 · 调换调用顺序 ｜ 改动最大

让节点先 `select()` 再问 `next_command()`，从根上避免「先置位再选点」。

| | |
|---|---|
| **优点** | 最干净 —— 状态机不必再表达「我刚决定要选点」 |
| **缺点** | `next_command()` 的返回值正是「该不该选点」的判据，调换后 `PASSIVE`/`DONE` 下也会白跑一次全图 BFS；同时把「是否该选点」的判断泄漏进了节点 |
| **结论** | 改动面远超这一条 bug 本身 |

---

## 六、G2 的建议：乙

根因是 `SENDING` 混淆了两种情形，**乙是唯一把这两件事分开的方案**，
而且新状态的语义能自解释 —— 下一个人读 `on_select_failed` 就知道该在什么时候调。

**甲可以当作接口冻结前的临时方案**（2 行，当天能上），
但它把「守卫放宽了」这件事留在代码里，需要一个注释说明为什么；
否则下一轮审查会再把它当成 bug 报一遍。

---

## 七、需要决定什么

1. **选甲还是乙** —— 这决定是否新增一个状态机方法
2. **`SENDING` 的语义定为哪个**：
   「正在发出」（甲需要放宽守卫来解释）还是「指令已发出、结局待报」（乙的自然产物）
3. 定了之后，**实现、类 docstring 的用法示例、那个名字与正文矛盾的测试要一起改到一致** ——
   三处对齐才算修完

---

## 八、给不了解背景的人：为什么现在提

这条不影响「返航成功率」（它只会让探索停住，不会让狗回不了家），
但它**直接影响「探索覆盖率」**，而且是静默的 —— 现象上只会表现为
「G2 好像不动了」，排查时会先怀疑地图、TF、Nav2，很难想到是状态机卡住。

现在是第 1 周、接口正在冻结，改一处状态机的成本最低。
