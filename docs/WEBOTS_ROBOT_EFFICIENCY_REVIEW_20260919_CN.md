# Webots 机器人效率 Review 与验收记录（2026-09-19）

## 目标与范围

本轮只接受同时满足以下两项目标的修改：

1. 机器人运行效率不退化：任务完成数、异常并发低进度、未解释 active
   低进度、原地转向、非计划停车和 clear-motion duty 同时过门禁。
2. Webots 模拟性能有保障：同场景、同 seed、无录屏 fast/no-rendering
   运行的 `sim_to_wall_ratio >= 1.0`，并报告 Supervisor 与联合规划 P99。

## 基线录屏与性能复测

- 配置：Scene C / FCFS / seed 44 / 180 s。
- 录屏：`results/webots_diagnostic_C_FCFS_20260919_190947.html`，配套
  JSON/X3D/CSS。
- 行为结果：`results/experiment_C_FCFS_20260919_190947.json`。
- 独立性能结果：`results/experiment_C_FCFS_20260919_191341.json`。
- 停车分析：
  `results/performance_workflow/current_baseline/stop_analysis.json`。

基线指标：tasks 1，route dispatch 224，audited replan 15，非计划停车
0，clear-motion duty 0.97651，未解释 active 低进度 36 段，原地转向
35 段，异常三车以上低进度 5.376 s，最小车距 0.54885 m，距离违规
0。无录屏倍率 1.01517x；Supervisor P99 79.43 ms，联合规划 P99
475.59 ms。

与 2026-09-15 Step 1 旧基线相比，异常并发低进度从 40.128 s 降到
5.376 s，dispatch 从 330 降到 224，replan 从 34 降到 15，
clear-motion duty 从 0.92236 提高到 0.97651。当前工作树中的首路线稳定性、
旧 predecessor 保留和陈旧 priority-yield 到达所有权修复确实改善了机器人
连续运行，但 180 s 仅完成 1 个任务，端到端吞吐仍未达到可宣称“完成优化”
的程度。

## 小步：自动化双跑 workflow

新增 `scripts/run_webots_performance_workflow.py`：

- 行为跑必须开启 Webots animation recording；
- 自动运行停车/低进度关联分析；
- 另起同配置无录屏运行测 Webots 性能；
- 只接受启动时间之后产生的结果，防止误读陈旧 JSON；
- 将动画、行为结果、性能结果、停车分析和最终 gate 放到单一 step 目录；
- 相对基线逐项检查任务数、motion duty、异常停车、未解释低进度、原地
  转向、非计划停车和最小距离；同时硬性要求零距离违规和倍率 >=1.0。

### Code Review R1：正确性与行为等价性

结论：PASS。

- 新文件只编排已有 runner/analyzer，不被 Supervisor 或 robot controller
  导入，不改变调度、路径、优先级、安全判断、事务提交或控制频率。
- 录屏跑与性能跑使用相同 scenario/scheduler/seed/duration；性能判断只读取
  无录屏结果，不会把动画导出开销归咎于 Webots。
- 新鲜度过滤和 `exist_ok=False` 防止拿旧结果或覆盖已验收证据。
- Review 发现单次同 seed 物理运行仍存在可观测波动，因此保留严格逐指标
  门禁；不因“代码未改控制逻辑”自动放行行为回退。

### Code Review R2：安全、性能与失败语义

结论：PASS（workflow 实现）；候选运行结论为 FAILED，未发布行为修改。

- workflow 不进入仿真 timestep 热路径，不增加机器人通信、规划调用或
  文件 I/O；只有验收时顺序启动两次 Webots。
- `try/finally` 恢复本进程修改的环境变量；任一运行失败、录屏缺件或结果
  不新鲜即失败，不回退到 standalone 或陈旧结果。
- 输出目录先创建且不覆盖，失败证据得到保留；重新验收必须使用新 step id。
- 安全门禁要求 `pair_distance_violations == 0`，并相对基线检查最小车距。
- 第二轮组合回归：245 passed + 4 subtests；全量回归：402 passed +
  4 subtests；`py_compile` 和 `git diff --check` 通过。

## workflow 实际 Webots 验证

Step：`workflow_tool_validation_20260919`。

- 录屏与全部证据：
  `results/performance_workflow/workflow_tool_validation_20260919/`。
- tasks 1；距离违规 0；最小车距 0.54885 m。
- 无录屏倍率 1.20387x，Supervisor P99 73.50 ms，联合规划 P99
  322.96 ms：Webots 性能门禁通过。
- 原地转向 35 -> 32：改善。
- 异常并发低进度 5.376 -> 5.856 s、未解释 active 段 36 -> 41、
  clear-motion duty 0.97651 -> 0.96142：相对行为门禁失败。

最终判定：`WORKFLOW_IMPLEMENTATION_PASS / WEBOTS_PERFORMANCE_PASS /
ROBOT_EFFICIENCY_CANDIDATE_FAILED`。本轮没有把失败录屏对应的控制行为作为
新优化提交，也没有以倍率提升掩盖机器人运动退化。

## 后续可接受的控制优化方向

当前第一热点仍是同步联合规划（P99 322--476 ms），行为热点是 36--41 段
未解释 active 低进度和 32--35 段原地转向。下一小步应只处理一种已由录屏
定位的原因，并通过本 workflow；不得重新引入此前已失败的全车曲率、自动跳
waypoint、重复 speed-scale 或普通 refresh 延长方案。
