# 冻结的历史验收实现

`validate_slam_frontier.py` 保留旧 Monitor、复杂人物、推车及旧故障注入的原始执行逻辑，仅用于显式历史复现。它不属于当前正式运行栈，不由普通启动或当前验收入口调用。

当前入口：`../run_navigation_runtime.py`。当前验收：`../run_runtime_trial.py` 或 `../validate_runtime.py`，只观察并下发任务命令。

历史复现必须在相应 runner 显式传 `--legacy-validation`，或手动设置 `SLAM_LEGACY_VALIDATION=1`；历史模式与正式 runtime profile 互斥。人物跟踪、动态障碍仍保持冻结，本轮不运行这些历史场景。历史阈值与算法没有迁移或调整，旧录包及失败结果保留。
