# 机器人效率修复 Workflow 验收记录

## 已通过并保留

### Workflow review gate

- 强制每步提供两份包含 `Verdict: PASS` 的实质 review；缺失即拒绝启动 Webots。
- 行为录屏和无录屏性能分跑，fresh artifact 原子归档。
- 同代码三次录屏证明存在有界时序波动，门禁改为噪声预算；行为修改仍必须至少
  一个核心 KPI 超出同一预算改善，安全和性能保持绝对门禁。
- 验证 step `wf_noise_gate_20260919`：行为精确复现基线，倍率 1.341x，PASS。

### Step A / B0：请求 provenance

- fresh-plan 请求新增 reason/class 统计；watchdog 进一步拆为 emergency、
  hard-motion、route-less、stale-wait。
- 两轮 review、407 项测试和 4 个 subtests、Webots 双跑通过。
- 180 s 结果：28 次 `joint_watchdog_emergency`、20 次
  `joint_watchdog_stale_wait`、10 次 physical progress lease；行为与基线一致，
  倍率 1.310x。

## 已失败并回滚

### Step B：首 endpoint 提前准备全事务 successor

- 两轮 review 和 407 项测试通过。
- 录屏、任务、dispatch、停车、转向、duty 与基线完全相同；无实质改善，FAILED。
- 已回滚并用 `step_b_rollback_20260919` 双跑确认恢复，倍率 1.777x。

### Step C：route-less 延迟 3.0 s -> 0.5 s

- 两轮 review 和 408 项测试通过。
- 录屏所有行为指标仍完全相同，说明该门限不是此次场景的有效触发点，FAILED。
- 已回滚并用 `step_c_rollback_20260919` 双跑确认恢复，倍率 1.831x。

## 录像位置/task 复核

- Robot 3 在约 132 s、task 9、前往 WS6 时明确处于
  `joint_window_endpoint` planned wait。
- Robot 2 在约 112 s、task 2、前往 WS5 时短时出现 active business state 但
  controller target 为空、`business_goal_or_inactive`；缩短 route-less 门限没有
  改变轨迹，说明该空窗未跨过 watchdog 采样门限或被更高优先级原因合并。
- 最终可操作主因收敛为 safety emergency 28 次和 stale validated-wait 20 次；
  普通 rolling refresh、首 endpoint 边沿和 route-less delay 均已被真实录像否证。

## 当前通过基线

`step_c_rollback_20260919`：tasks 1，dispatch 224，replan 15，未计划停车 0，
clear-motion duty 0.97651，未解释 active 低进度 36，异常并发低进度 5.376 s，
原地转向 35，最小距离 0.54885 m，距离违规 0，无录屏倍率 1.831x，Supervisor
P99 37.65 ms，joint planning P99 246.83 ms。

下一行为修改必须先把 28 次 emergency 按机器人位置、peer pair、TTC、目标线段和
task 拆分；安全请求不可直接节流。只有能证明是假阳性或重复边沿时才允许修改，
且仍执行两轮方案 review、两轮 code review、录屏和无录屏性能门禁。
