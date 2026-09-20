# 机器人效率修复强制 Workflow

状态：ACTIVE。

## 每一小步的固定顺序

1. 冻结上一个通过版本的 `workflow_summary.json`。
2. 一个 step 只修改一个行为变量；观测和 workflow 修改也单独成 step。
3. 运行定向测试、相关组合测试和全量测试。
4. Code Review R1：正确性、根因匹配、业务语义、输入输出兼容性。
5. Code Review R2：安全、事务/状态机、热路径性能、失败恢复、回滚性。
6. 两份 review 都必须包含 `Verdict: PASS`；否则 runner 拒绝启动 Webots。
7. 行为跑：Scene C / FCFS / seed 44 / 180 s，强制 Webots animation recording。
8. 对同次 animation JSON/X3D 和实验 JSON 执行 stop analysis。
9. 性能跑：完全相同配置，关闭录屏，fast/no-rendering。
10. workflow 将 review、动画、行为 JSON、性能 JSON、停车分析和 gate 原子归档。
11. 任一门禁失败：标记 FAILED，保留证据，只回滚本 step；根据录屏修订下一方案，
    修订后的方案重新做两轮 review。

## 硬门禁

- `tasks_completed` 不下降；
- `abnormal_mass_stop_seconds`、`unexplained_active_intervals`、
  `rotating_in_place_intervals`、`unplanned_task_stops` 均不增加；
- `clear_motion_duty_cycle` 不下降；
- `pair_distance_violations == 0`，`min_pair_distance` 不下降；
- 无录屏 `sim_to_wall_ratio >= 1.0`；
- Supervisor/joint planning P99 不得出现新的显著退化。

## 实施队列

- WF：runner 强制两份 review 证据。
- A：联合规划请求原因 provenance；先只观测，不改变规划结果。
- B：基于实测 waypoint/index/reservation 剩余量的 pre-plan eligibility gate。
- C：successor 近端方向收益门，仅在 B 通过后评估。
- D：联合规划分时 job，仅在行为门稳定后实施。

## Workflow Review R1

范围：顺序、证据绑定、门禁完整性、失败语义。

- review 在 Webots 前执行且复制进 step 目录，避免事后补写审查结论。
- 行为与性能分跑，避免录屏开销污染 Webots 性能结论。
- 使用启动时间筛选 fresh artifacts，禁止读取旧结果。
- 相对上一通过版本逐项比较，不以平均值掩盖单项机器人效率回退。

Verdict: PASS

## Workflow Review R2

范围：安全、可恢复性、热路径影响、可复现性。

- runner 不被 controller 导入，不进入 timestep 热路径。
- step 目录不可覆盖；失败证据保留，新尝试必须使用新 step id。
- 安全距离和零违规属于硬门禁，不能用 throughput 或倍率抵消。
- 每个行为修改独立录屏；失败时只反向撤销该步补丁。
- 当前单 seed 可能有物理波动，但严格门禁宁可产生假阴性，也不会放行已观测回退。

Verdict: PASS
