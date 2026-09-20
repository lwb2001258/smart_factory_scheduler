# 机器人避障、路线预约与联合规划实施 Workflow

## 1. 目标、依据与适用范围

本 workflow 用于落实 `ROBOT_COLLISION_AVOIDANCE_ROUTE_RESERVATION_REVIEW_CN.md` 中已经确认的问题和待验证风险。范围包括安全配置、静态几何、联合时空模型、预约生命周期、动态风险盾、交通规则、恢复机制、可观测性和 Webots 验收。

安全优先于吞吐和“零重规划”。`ZERO_REPLAN_600S_WORKFLOW_CN.md` 的零重规划目标只能作为优化目标；如果安全盾为避免碰撞必须触发可审计的重规划，不得为了满足零计数而禁用安全动作。

本 workflow 不授权瞬移恢复。所有正式 Webots 门禁必须设置 `SMART_FACTORY_ENABLE_NONPHYSICAL_RECOVERY=0`。

## 2. 不可跳过的质量门禁

每个步骤必须依次执行以下闭环，缺少任一证据时状态只能是 `BLOCKED` 或 `IN_PROGRESS`，不能标记 `PASS`：

1. 冻结本步骤输入、修改范围、失败指标、回滚点和预期不变量。
2. 实现最小变更及针对性测试。
3. Code Review R1：正确性与安全不变量；修复全部阻断项。
4. Test R1：针对性测试、语法/导入检查；保存命令、退出码和摘要。
5. Code Review R2：状态所有权、并发时序、epoch/version、失败与回滚路径；修复全部阻断项。
6. Test R2：相关子系统回归和性质/边界测试；保存证据。
7. Code Review R3：完整 diff、跨模块语义、性能、可观测性、配置兼容和测试充分性；修复全部阻断项。
8. Test R3：完整 `python -m pytest -q`；不得用局部测试替代。
9. Webots gate：先执行规定短时 smoke，再执行该步骤规定时长；解析 JSON 和 stdout/stderr，人工观察不能替代指标。
10. 将三轮 review、三轮测试、Webots 输入/输出、发现和修复写入验收日志。Webots 失败时返回本步骤第 2 项，并重新完成三轮闭环。

三轮 review 必须基于三个不同关注面，不允许复制同一结论：

- R1：算法/几何正确性、安全边界、单位和阈值。
- R2：消息乱序、陈旧状态、事务原子性、重启、回滚和预约生命周期。
- R3：全量 diff、性能退化、日志/指标真实性、环境覆盖和运维可解释性。

## 3. 统一证据格式

每一步在 `ROBOT_COLLISION_AVOIDANCE_IMPLEMENTATION_ACCEPTANCE_LOG_CN.md` 中追加：

```text
step/status/date
reviewed_commit + dirty + changed_files + file_hashes
resolved_flags + world_hash + Webots/Python version
R1 findings/fixes/verdict
Test R1 command/result/artifact
R2 findings/fixes/verdict
Test R2 command/result/artifact
R3 findings/fixes/verdict
Test R3 command/result/artifact
Webots command/scenario/scheduler/seed/duration/result/log
gate metrics + first failure evidence + rollback decision
```

不得覆盖旧失败结果；失败是审计证据。生成物统一放在 `results/collision_workflow/<step>/<run_id>/`。

统一命令模板（每轮必须把实际展开后的命令写入日志）：

```powershell
# Test R1：按步骤选择针对性文件，并补充本步骤新增测试
python -m pytest -q tests/test_joint_grid_planner.py tests/test_motion_coordination.py

# Test R2：导航安全相关回归
python -m pytest -q tests/test_joint_grid_planner.py tests/test_motion_coordination.py `
  tests/test_supervisor_prediction.py tests/test_robot_protocol.py `
  tests/test_metrics_safety.py tests/test_priority_yield_resume.py

# Test R3：全量回归
python -m pytest -q

# Webots；运行前按步骤设置 SMART_FACTORY_SIM_DURATION 和证据目录
python scripts/run_experiments.py --scenario C --scheduler FCFS `
  --seed-values 42 --webots "C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
```

`run_experiments.py` 当前将结果写入仓库 `results/`。每次运行前后必须记录目录快照，只接受本次启动后新生成且 JSON 内 scenario/scheduler/seed/duration 匹配的文件，再将结果和日志复制到证据目录。不得用“目录中已有一个通过结果”代替本次运行。

回滚规则：本仓库可能包含用户的未提交修改，禁止使用 `git reset --hard`、`git checkout --` 或覆盖整个文件。每一步开始时保存目标文件 hash 和局部补丁；失败时只通过审阅后的反向补丁撤销本步骤新增修改。与用户既有修改重叠且无法安全分离时，本步骤标记 `BLOCKED`，不得擅自回退。

## 4. 全局安全与活性门禁

除 Step 0 的基线捕获例外外，所有实施步骤的 Webots gate 都必须满足：

- `pair_distance_violations == 0`，且 `min_pair_distance >= 0.50 m`；
- `nonphysical_recoveries == 0`；
- 无 controller 异常退出、世界加载 ERROR、NaN/Inf 路径或部分 transaction 激活；
- 所有路线写入具有 source、robot、epoch/version、时间和原因；
- 所有安全停止具有触发证据，陈旧 peer pose 不得被当作新鲜状态继续高速执行；
- 相比冻结基线，完成率和吞吐退化超过 10% 时必须标记 `SAFETY_PASS_PERFORMANCE_FAIL` 并给出归因。它不阻止后续安全步骤继续修复，但阻止 Step 5 最终发布；最终验收必须同时关闭安全和性能缺口；
- 不用平均值掩盖单 seed 违规；任一 seed 安全失败即整步失败。

正式最终验收额外要求 Scene C 至少 10 个固定 seed、每个 1800 仿真秒。前置步骤使用较短门禁以控制反馈时间，但不能替代最终验收。

## 5. 实施步骤

### Step 0：冻结可复现基线和审计工具

实现：

- 记录 Git commit、dirty 状态和关键文件 SHA-256；
- 记录最终解析的安全开关、阈值、world hash、Python/Webots 版本；
- 固定 Webots 输入 manifest、场景 C、FCFS、8 机器人和基线 seed；
- 建立结果审计器，校验 run mode、时长、seed、非物理恢复、安全距离、关键计数和输出新鲜度；
- 区分 rolling refresh、business transition、emergency replan、escape-triggered replan 和 transaction retry。

测试：provenance 缺失、错误 seed、短运行、旧结果复用、NaN、计数不一致均必须 fail closed。

Webots：30 s 启动 smoke + 90 s 基线；本步只冻结现状，不要求改善 replan/escape。若出现安全违规，状态记录为 `CAPTURED_UNSAFE_BASELINE`，保留首个失败证据后允许进入 Step 1 修复；不得写成安全 PASS。若启动、结果身份、provenance 或指标审计失败，则状态为 `BLOCKED`，不得继续。

### Step 1：统一 MotionSafetyConfig 与静态几何

实现：

- 建立唯一版本化 `MotionSafetyConfig`，统一半径、载荷外形、最大实测速度、减速度、控制/通信延迟、hard/planning/prediction clearance 和 stale age；
- 停止距离由参数计算，并在启动时打印解析值、来源和 fingerprint；
- OccupancyGrid、JointGridPlanner、Supervisor、RobotController 和指标审计引用同一契约；
- `_segment_clear()` 复用统一障碍几何 API；
- endpoint relaxation 改为最小 connector 或显式 terminal approach corridor，禁止固定大方形无约束放松。

测试：单位、边界等号、配置覆盖、地图一致性、endpoint connector、充电站/工位可达和障碍不可穿越。

Webots：30 s smoke + 150 s Scene C；0 安全违规、所有 dock/charging approach 可达、无新静态碰撞或规划不可达激增。

### Step 2：联合时空模型与预约生命周期

实现：

- 明确维持四邻域；删除不可达对角分支和死代码，除非完整实现 corner-clear 与 validator 配套语义；
- 用 Webots 轨迹样本标定直行、90°/180°转向、短 waypoint、降速和 hold 的 P95/P99 traversal time；
- slot/动作持续时间覆盖 P99、控制与通信 buffer，纳入旋转 swept occupancy；
- 修正 soft tier：名称、注释、实际生效阈值和 `is_relaxed` 必须一致；
- reservation 根据实测进度只向未来保守延长；处理提前、滞后、偏航、失联、重启和释放；
- 新旧滚动窗口 transaction 提交前验证 prefix/tail 连续时空安全；slot 0 冲突 fail safe。

测试：动作持续时间、旋转占用、迟到延长、提前释放、重启、失联、窗口衔接、slot 0、阈值等号和 reservation 原子回滚。

Webots：30 s smoke + 300 s Scene C；窗口断档/部分激活为 0，安全违规为 0，预约过期后仍移动为 0。

### Step 3：CPA/TTC 动态安全盾与 stale-pose fail-safe

实现：

- 将 `collision_safety.assess_motion_risk()` 接入 Supervisor；输入测量速度、heading/angular velocity、speed scale、hold、时间戳和误差带；
- CLEAR/CAUTION/BRAKE/EMERGENCY 映射为明确且可审计动作；
- caution age 降速、stop age 停车；乱序 pose 不得刷新新鲜度；
- RobotController 保留独立硬安全层，阈值由安全契约派生；
- 冲突 component 内外均复验，任何命令失败执行原子回滚或协调安全停车。

测试：CPA/TTC、追尾、迎面、交叉、静止 peer、乱序/丢包、制动可达性、组外二次冲突、命令失败回滚。

Webots：30 s smoke + 450 s Scene C，并执行 pose 延迟/控制器慢帧故障注入；安全违规为 0，陈旧数据继续额定速度为 0。

### Step 4：交通规则、staging、候选选择与恢复分级

实现：

- 将中央交叉口、货架窄口、dock 入口和共享 dock 建模为有容量的 zone resource；
- staging 采用完整增量路程、安全余量和路线侧别评分，先搜索 0.75～1.5 m；增加分配粘性和 primary lease；
- `JointGridPlanner` 的类契约与实现一致；在严格预算内比较多个 deterministic priority order，使用总距离、等待、最大个体绕行和路线切换综合评分；
- 路径滞回只有在旧路线不安全、持续无进展或新路线显著更优时才替换；
- `detour_ratio` 仅作为与 extra distance/TTC/等待联合的标定门禁；
- right-of-way 含 aging 上限；escape 按速度错峰、等待、局部重规划、路线附近避让、全方向逃逸逐级升级并限频。

测试：共享 dock、身份稳定、zone 容量、priority fairness、短 direct path、唯一安全绕行、路径抖动、escape 冷却和 conflict component 升级。

Webots：30 s smoke + 600 s Scene C；安全违规为 0，每台机器人有净进展；P95 detour、走廊切换、escape/replan 相对 Step 0 基线显著下降且吞吐不退化。

### Step 5：规划器演进和最终验收

实现：

- 保留 prioritized planner 为快速首选；首选失败时尝试有限确定性顺序或局部 CBS；
- CBS/RHCR 只能在异步 worker 或跨 timestep 有预算增量执行；同步控制循环不得被长搜索阻塞；
- 所有候选必须通过统一 validator 和 joint transaction；过期 candidate 自动作废；
- 输出 bottleneck cell、order sensitivity、timeout/no-solution 和候选成本诊断。

测试：超时取消、旧候选、worker 失败、顺序敏感、局部 CBS、validator 拒绝、transaction rollback 和同步循环预算。

Webots：先 900 s 预验收；通过后 Scene C、FCFS、8 机器人、至少 10 个固定 seed × 1800 s。必须逐 seed 满足全局安全门禁；报告 P50/P95/P99 detour、最差单机器人延误、吞吐、完成率、replan、escape、planned wait、TTC 和公平性。

### Step 5A：连续运动调度与 stop-and-go 消除（新增，不替换原步骤）

目标：机器人在没有静态阻挡、动态风险、资源冲突、业务停靠或有效安全停车理由时应连续高效运动。Webots 渲染卡顿不得与控制器主动零速混为一谈；验收以 controller command、pose 和预约事件为准。

设计约束：

- `joint_time_slot_s=4.75 s` 只作为保守 latest-exit/预约失效上限，不得作为每个 0.25 m cell 的固定行驶节拍；
- 控制器不得自行提前进入未授权 cell。Supervisor 仅在统一 reservation validator 证明目标 vertex、反向 edge、旋转 swept occupancy 和 zone 均安全后，签发带 `plan_epoch + waypoint_index + grant_seq + valid_until` 的 advance grant；
- grant 只允许当前 epoch 的下一个 waypoint，乱序、重复、过期、跨 epoch grant 均拒绝且不得刷新新鲜度；joint transaction 回滚/替换时原子撤销旧 grant；
- 只有状态为 `activated` 的 joint transaction 可以签发 grant；新 epoch 激活时 RobotController 原子清空全部旧 grant，同 epoch 只接受严格递增 `grant_seq` 且 grant index 必须等于当前 next waypoint。grant 消息丢失或身份不明时默认停车，不依赖可能丢失的 revoke ACK 保证安全；
- 已授权的连续安全前缀不得因 2 s rolling refresh 被重置。只有旧路线不安全、业务目标变化、持续无进展或候选显著更优时才替换；
- CAUTION/BRAKE/EMERGENCY、stale pose、预约冲突、zone 容量和 RobotController 独立硬安全层保持最高优先级，运动效率优化不得降低任何安全距离；
- 先增加命令级可观测性再改变运动语义，禁止用吞吐或平均速度推断“没有无理由停车”。

实施步骤（每一步至少两轮 code review 和两轮验证）：

1. **5A-1 可观测性**：RobotController 上报最终左右轮命令、线/角速度、active epoch/index、wait reason、risk level 和 grant identity；MetricsCollector 记录 eligible-navigation、moving、turning、valid-conflict-wait、unblocked-zero-speed 和 stop/go transition，并执行事件—汇总对账。
2. **5A-2 预约 grant**：在现有 joint plan 上计算可连续释放的安全前缀；Supervisor 是唯一 grant writer，提交前复验当前 pose/epoch/index 和所有内外 component reservation；命令发送失败时保持旧窗口和安全停车。
3. **5A-3 控制器消费**：RobotController 只消费严格递增且身份匹配的 grant；把 `joint_time_slot_s` 从固定 pacing 改为 deadline guard；到达后若下一 cell 已获 grant 则连续行驶，否则以明确冲突证据停车。
4. **5A-4 rolling 连续性**：保留已授权、尚安全的 active prefix，抑制等价路径重发和 waypoint/index 重置；新候选仍经统一 validator 和 joint transaction。
5. **5A-5 门禁与返工**：任何安全、身份、计数、性能或连续运动门禁失败均返回相应实现步骤，并重新完成该步骤两轮 review/test；Step 5 最终三轮 review/test 要求保持不变。

新增测试：无 peer 直线路径连续运动、转弯不停顿、真实 vertex/edge/zone 冲突停车、grant 乱序/重复/过期/跨 epoch、transaction rollback 撤销 grant、rolling refresh 不重置前缀、stale pose 仍停车、命令失败 fail-closed、组外二次冲突、指标对账和同步循环预算。

新增运动品质硬门禁：

- `unblocked_zero_speed_seconds`：排除业务停靠、目标到达、有效 BRAKE/EMERGENCY、stale-stop、已审计 vertex/edge/zone 冲突和必要原地旋转后，连续零线速度超过 0.25 s 的时间；全局占 eligible-navigation 比例 `<= 3%`，任一机器人 `<= 5%`；
- `unblocked_stop_events`：上述状态持续超过 0.50 s 记一次；任一机器人 `<= 0.5 次/active-minute`，且不得出现固定 slot 周期峰值；
- `clear_motion_duty_cycle`：CLEAR 且有剩余路径、无有效等待理由时，线速度不低于 0.03 m/s 或正在必要原地旋转的比例；全局 `>= 95%`，任一机器人 `>= 90%`；
- `joint_slot_release` 等待必须携带具体 peer/resource、epoch/index、预测占用区间和 validator 证据；缺失证据一律计为 unblocked stop；
- 最终轮速等连续量只随现有 controller status 定频上报，Supervisor 在线 O(1)/robot 累积 duration；只在状态转换时追加事件，禁止逐 timestep 写事件或同步刷盘。指标 JSON 必须同时保存采样周期、丢包数和最后序号；丢失区间不得被当作 moving/CLEAR；
- 保持原安全门禁、`sim_to_wall_ratio >= 1.0`、调度 P99 `<= 50 ms`；相对 Step 4 通过基线，吞吐/完成率不得退化超过 10%，planned-wait seconds 和 stop/go events 必须显著下降；
- Webots：30 s 无冲突直线专项 + 90 s 多机器人 smoke + 600 s Scene C 回归 + 900 s 预验收；全部通过后才允许重新启动 10 fixed seeds × 1800 s 最终验收。
- 性能回滚点：每个 5A 子步骤保留目标文件 hash/局部补丁；若 `sim_to_wall_ratio`、controller status 丢包或调度 P99 失败，只反向撤销该子步骤新增补丁，不覆盖用户既有修改。每轮 Webots 必须使用新 JSON 和本次启动时间校验。

## 6. Workflow 自身复审要求

开始 Step 0 前必须完成至少两轮 workflow review，并记录在验收日志：

- Workflow Review 1：需求覆盖、步骤依赖、可执行命令、证据可重建性和回滚。
- Workflow Review 2：安全/活性冲突、门禁可测量性、Webots 成本、失败继续策略和是否存在以指标代替真实行为的问题。

两轮均修订完成并明确 `PASS` 后才能执行 Step 0。

## 7. 完成定义

只有 Step 0 具备可审计的 `CAPTURED_BASELINE`/`CAPTURED_UNSAFE_BASELINE` 证据，Step 1～5 均具备三轮 review、三轮测试和 Webots PASS 证据，且最终 10×1800 s 全部安全与性能门禁通过，才能称 workflow 完成。短时 smoke、历史结果、单元测试或单 seed 平均值均不能替代最终验收。若受运行时间或外部环境限制尚未完成，必须报告当前步骤、已完成证据和下一项未满足门禁，不得宣称已实现全部方案。
### Step 5B：无物理阻塞的长期静止消除（新增，不替换 Step 5A）

目标：任何拥有有效业务/回充/回家目标的机器人，即使 rolling epoch、route version、partial endpoint 或 recovery 状态持续变化，也不能在没有审计物理冲突的情况下无限静止。

设计约束：

1. 为每台 active robot 建立独立于 `plan_epoch/path_version` 的 `physical_progress_lease`。租约只允许被测量位移、沿目标方向的净进展或已完成业务 waypoint 重置；prepare/arm/activate、route rewrite、hold、grant/deny、replan request 均不得重置。
2. 租约包含 `lease_generation/start_time/start_position/best_goal_distance/last_progress_at/escalation_stage`；所有更新单调，乱序状态包不得倒退。
3. grant 的 stale-peer 拒绝只适用于可能进入当前/下一段扫掠管、目标顶点、反向边或 turning zone 的相关 peer。与该段最小距离大于 `planning_clearance + stale uncertainty bound` 的 stale peer 不得造成全局授权饥饿；相关 stale peer 仍默认停车。
4. `stationary/empty candidate`、`joint_window_endpoint`、`joint_advance_grant`、重复 0.5 s hold 跨 epoch 累计。到达软期限先立即请求 fresh joint plan；到达硬期限仍无测量进展时，执行既有 validated physical escape；禁止以新 epoch 续期硬期限。
5. recovery/escape 成功的判据是测量位移或目标距离改善，不是命令发送成功。失败后保留租约并继续升级，不能清空 stall clock。
6. active robot 每次零速状态必须携带可审计 `control_stop_reason`：`validated_conflict/stale_relevant_peer/partial_endpoint/supervisor_hold/emergency/business_goal/unknown`。`unknown` 不能获得合法等待豁免。

实施步骤（每步至少 2 轮 code review 与验证）：

- 5B-1：增加跨 epoch physical-progress lease 与 stop reason 遥测，不改变运动决策。
- 5B-2：把所有 stall/reset 路径改为“仅物理进展重置”，增加软/硬期限单调升级。
- 5B-3：将 stale-peer grant 校验收敛到当前/下一段相关空间，保持相关 stale peer fail-safe。
- 5B-4：stationary candidate、partial endpoint、grant denial 统一接入有界等待；fresh replan 无进展后进入 validated physical escape。
- 5B-5：30 s/90 s smoke、600 s Scene C、900 s 预验收；所有 active robot 在无审计 block 时不得连续静止超过硬期限，`unknown` 长停顿为 0，并保留 Step 5A 全部安全/连续率/吞吐门禁。通过后才恢复 10 fixed seeds × 1800 s。

回滚：任一修改造成安全距离违规、未授权运动、相关 stale peer 下继续运动、非物理恢复、吞吐较 Step 5A 已通过的 600 s 结果退化超过 10%，立即回滚该子步骤并保留失败证据。

#### Step 5B 性能返工：soft lease 去抖与阈值标定

- 600 s 证据表明 3 s soft deadline 会把正常长转向/低速接近误判为需要立即滚动重规划，造成 replan request 放大和吞吐退化；该结果必须记录为 `SAFETY_PASS_PERFORMANCE_FAIL`。
- soft deadline 调整为 5 s，hard deadline 保持 8 s；5 s 只触发一次边沿式 fresh-plan 请求，route/epoch 变化不得续期，真实物理进展才可重新武装。
- 5～8 s 仍提供 3 s 的安全规划窗口；8 s 无测量进展继续进入 validated physical recovery，不能为性能关闭硬期限。
- 重新执行两轮 code review/test、90 s、600 s 和 900 s Webots；600 s 吞吐必须不低于 Step 5A 同 seed 基线 1.62 tasks/min。

#### Step 5B 最终耐久返工：命令零速租约

- 1800 s seed 42 暴露出 Webots 接触抖动/被动位移可刷新 physical-progress lease，而 controller 实际连续输出零轮速 36～50 s；因此物理进展租约不能单独证明机器人被主动调度。
- 增加跨 epoch 的 `uncommanded_zero_lease`：对 active navigation、连续零轮速且没有有效 validator evidence 的状态 fail-closed 计时（包括 `local_planner_zero_replan/unknown/route_exhausted` 及证据缺失的 grant wait）；route、epoch、replan、被动位移不得清零。
- 只有观测到非零线/角轮速、业务导航结束或携带有效 validator evidence 的安全等待才可清零；连续 8 s 后触发一次 validated physical recovery，恢复动作必须在观测到非零轮速后才可重新武装。
- 该租约与 physical-progress lease 互补：前者捕获“没有运动命令”，后者捕获“有运动命令但没有物理净进展”；任一硬期限到期均可进入同一安全恢复门。
- 重新完成三轮方案 review、每步两轮 code review/test，并从 30/90/600/900 回归后重新启动 10×1800 s；任何 seed 出现无验证阻塞的 >8 s 零轮速均失败。

##### command-zero 响应预算标定

- 8 s 是外部可观测零轮速上限，不是开始计算恢复的时刻；watchdog 扫描、规划派发和控制器接收必须包含在该预算内。
- command-zero recovery trigger 设为 7 s，预留最多 1 s 端到端响应；physical-progress hard deadline 仍为 8 s，所有安全 validator 不变。
- 若 7 s 触发后仍出现 >8 s 无验证零轮速，继续判 FAIL，不得用采样周期豁免。

##### command-zero 恢复确认与限频重试

- physical recovery 的“派发成功”不等于恢复成功；派发后 2 s 仍无非零轮速时必须重新进入 validated recovery，禁止永久 latch。
- 每次重试仍执行完整 peer/grid/segment/zone validator；重试间隔不得短于 2 s，不得逐 timestep 改写路线。
- hard-zero deadline 事件每个连续 episode 只记一次，恢复尝试继续由 escape/replan events 审计；只有观测非零轮速、业务结束或有效安全等待才结束 episode。

##### command-zero 最终响应余量

- 900 s 实测表明 7 s 触发后的 validated plan/transaction/controller 生效最长约 1.59 s，因此 1 s 余量不足。
- command-zero trigger 调整为 6 s，为外部 8 s 上限保留 2 s 端到端余量；2 s recovery retry 间隔、physical hard=8 s 和所有 validator 保持不变。

##### 连续停车 episode 边界真实性

- `command_motion_changed_at` 只描述电机最后一次从非零变零，不能跨越中间的 validated wait/emergency/缺测区间作为新 unblocked episode 起点。
- 新 unblocked-zero episode 起点必须取 `max(command_motion_changed_at, current_contiguous_interval_start)`；任何合法等待或不可分类区间都切断 episode。
- command-zero lease 与原始 status 不变；该修复只纠正审计持续时间，不隐藏或缩短真实连续无证据零速。

#### Step 5B 最终状态（2026-09-12）

- 方案在实现前已完成三轮 review；command-zero 响应预算、恢复确认/限频重试和 episode 边界三次后续标定也分别完成三轮 review。
- 每项实现返工均完成至少两轮 code review 和验证；最终 focused suite 为 150 passed + 4 subtests，全量 suite 为 298 passed + 4 subtests。
- 30/90/600/900 s 递增 Webots 门槛通过后，完成 seed 42--51 的 10×1800 s 最终验收；最长无验证零速 7.328 s、unknown stop=0、距离违例=0，最低仿真倍率 1.348×，最高调度 P99 6.379 ms。
- 状态：`PASS / WORKFLOW COMPLETE`。原 Step 0--5 与 Step 5A 的要求均保留；本状态表示新增 Step 5B 也已关闭，不降低任何安全、吞吐或 Webots 性能门槛。

### Step 5C：目的地优先清场与原子交接（提案，待用户确认后实施）

目标：工作站、仓储取放点、充电站及其他容量受限目的地完成当前服务后，原占用机器人必须优先、安全地驶出目的地区域；后续机器人在目的地外等待，不能因普通任务优先级较高而先占用出口，形成“进入者等待清场、占用者又因低优先级无法离场”的循环。该步骤只增加 terminal resource 仲裁，不改变 FCFS/RL 的任务分配结果、普通道路优先级或既有安全 validator。

#### 两轮方案 review 后的设计约束

1. 每个任务目的地建立容量为 1 的 `terminal_zone`、带滞回的 `clearance_boundary`、至少一个入口 staging 和一段最短安全 egress prefix；充电站使用同一资源协议，不另建绕过安全门禁的特殊路径。
2. owner 由 fresh measured pose 与服务状态共同决定，不能只由逻辑 goal、route 或 reservation 推断。pose stale 时禁止释放 owner 和签发 inbound grant；物理占用事实高于任务优先级。
3. 状态机固定为 `OCCUPIED -> SERVICE_COMPLETE -> CLEARING_TERMINAL -> PHYSICALLY_CLEAR -> RELEASED -> NEXT_INBOUND_GRANTED`。必须先观察到机器人越过 clearance boundary 的连续 fresh 样本，才能释放目的地；禁止在“已规划/已发命令/已分配新任务”时提前释放。
4. `CLEARING_TERMINAL` 机器人只在 terminal zone、出口冲突域和经过统一 validator 的有限 egress prefix 内获得 evacuation priority；越过边界后立即恢复原任务优先级。该权限不能传播到普通走廊，不能改变新任务的 scheduler priority。
5. 到达机器人的最终入口 cell 与 owner 的 egress prefix 互斥。owner 清场前，incoming robot 只能获得到 staging 的路线和携带 terminal owner/version 的有效等待证据；不允许其高优先级路线占用出口或压入 clearance boundary。
6. 原子交接使用单调 `terminal_epoch + owner_id + handoff_seq`。顺序必须是先提交并验证 egress prefix、再确认物理清空、最后签发唯一 inbound grant；乱序、重复、过期、跨 epoch 或 owner 不匹配的消息全部拒绝。
7. 在取货点完成 pickup 后，驶向 delivery 的首段就是清场 egress；完成 delivery 后，无论是否立即链式分配新任务，都先完成清场前缀，再衔接新任务。允许提前保存 assignment，但不得让新任务普通优先级覆盖清场权。
8. 充电中的机器人继续持有 owner，不能被驱逐；达到现有充电完成条件或主动结束充电后进入 `CLEARING_TERMINAL`。等待充电的低电量机器人可沿用现有队列优先级，但必须在 staging 等到 `PHYSICALLY_CLEAR` 后才获准进入。
9. 如果 egress 被真实物理冲突阻塞，保持 BRAKE/EMERGENCY/stale-peer fail-closed，并接入现有 5B physical-progress 与 command-zero watchdog；timeout 只能请求重新验证的 egress/recovery，不能直接释放 terminal 或允许双方同时进入。
10. terminal 仲裁只在状态转换时写审计事件，常规检查保持 O(1)/terminal 或使用现有局部冲突 component；不得逐 timestep 刷盘、全局重规划或降低 Webots 仿真倍率。
11. 任一机器人只要以 fresh pose 位于任一工作站、仓储点或充电站的 departure vicinity 内，且其当前业务目标不是该目的地并具有有效离场路线，就必须在当前局部冲突 component 中获得最高离场优先级；该保障不依赖 service-complete/owner lease，避免状态丢失或 lease 已释放后仍停在目的地附近。
12. vicinity 离场优先级只改变与该机器人实际冲突的 winner/yielder，不进入 scheduler priority，也不提升无关路口或全局 joint order；BRAKE、EMERGENCY、stale pose、距离、swept segment、reservation 与 zone validator 始终高于离场优先级。两台同时离场时沿用现有确定性优先级，不允许互相无限抢占。

#### 实施 workflow（未获确认前禁止执行代码修改）

- **5C-0 基线与可复现证据**：从 Webots 记录提取“完成服务后仍占 terminal、获得新任务、incoming 到达、双方等待/恢复”的时序；补充一个确定性的双机器人工作站场景和一个充电站场景。只建立基线，不改变决策。
- **5C-1 资源与遥测**：定义 terminal zone/boundary/staging/egress 数据；增加 owner、terminal epoch、服务状态、清场状态、inbound wait reason 和 handoff 事件—汇总对账。此步不得签发新权限。
- **5C-2 owner 状态机**：实现基于 fresh pose 的 owner 获取、服务完成、带滞回的物理清空与 fail-closed 释放；覆盖 pickup、delivery、charge-complete 和 task chaining。
- **5C-3 evacuation priority**：在现有 priority-yield/joint candidate/grant validator 前增加局部 terminal 仲裁；为 owner 提交有限 egress prefix，把 incoming 截止在 staging，并在离开边界后恢复既有优先级。
- **5C-4 原子 handoff 与充电站**：加入 terminal epoch/handoff sequence、乱序拒绝和唯一 inbound grant；统一充电完成清场、等待队列、低电量优先与异常退出语义。
- **5C-5 恢复与故障注入**：验证 egress 无解、controller 命令失败、pose stale、owner 重启、rolling epoch 替换、recovery 与 task cancel；所有失败保持 owner，不得提前释放或产生双重授权。
- **5C-6 递增验收**：先跑静态/协议单测与场景测试，再执行 30 s 单 terminal、90 s 双机器人、600 s Scene C、900 s 预验收；全部通过后才允许重跑 10 fixed seeds × 1800 s。每个实施步骤至少完成两轮 code review 和两轮验证，失败证据必须保留并返回对应步骤。
- **5C-7 任意目的地 vicinity 离场保证**：从每 timestep 已有 fresh pose 派生最近 terminal departure context；对“目标不是当前位置 terminal 且有有效离场路线”的机器人施加 pair-local 最高离场权，覆盖 owner/lease 缺失、task chaining、充电完成及第三方阻挡。增加工作站/仓储点/充电站、approaching/non-navigating/stale/outside-vicinity、双 departure 和性能回归测试；重新执行两轮 code review/test 与完整 Webots 递增/最终门禁。
- **5C-8 规划失败离场重试闭环**：目的地服务完成后，即使暂无新任务、assignment/pickup-to-delivery/idle relocation 首次规划失败，也必须保留独立 egress retry session。按 0.5/1.0/2.0 s 有界退避依次重试业务目标、可用休息点和局部安全 egress 候选；成功提交后只由 fresh pose 越界结束，失败不得静默 `continue`。pickup 服务事实必须在 delivery 规划前提交，避免失败后仍伪装为“正在前往已到达的 pickup”。每次失败/重试/提交/物理完成须 transition-only 审计，并接入 5B watchdog；每个子步骤至少两轮 code review/test，之后重新执行 90/600/900/10×1800 s Webots 门禁。
- **5C-9 同目标 rolling route 稳定提交**：对 activated joint transaction 记录激活时 measured pose。同一业务目标且无新冲突、安全升级或 liveness request 时，只要任一成员尚未产生至少 0.05 m 物理进展，就保留当前已验证路线，不得由固定 2 s rolling tick 覆盖 epoch/waypoint；全部成员已开始运动、前缀完成、目标变化，或 5B soft/hard watchdog 明确请求后才允许替换。该步骤不改变 FCFS、交通优先级、规划目标、路径评分或任何 validator，只修正已验证路线的生命周期；每个实现子步骤至少三轮 code review 和三轮测试，再执行完整 Webots 门禁。
- **5C-10 物理恢复路线执行租约**：hard watchdog 成功提交 `_command_reverse` 等已验证物理恢复路线后，为该机器人建立独立于普通 rolling epoch 的 recovery execution lease。普通 joint refresh 不得在机器人到达恢复终点或租约超时前覆盖该路线；同时不得冻结其余机器人，联合候选必须保留恢复机器人的既有时空轨迹/占用作为约束。目标变化、BRAKE/EMERGENCY、stale pose 和更高等级安全动作仍可立即中断；超时必须回到既有 watchdog 重新验证，禁止永久锁定。该步骤不改变 scheduler、priority、路径评分、冲突规则或 validator。方案和 workflow 各三轮 review；每个实现步骤至少三轮 code review 和三轮测试，再执行完整 Webots 门禁。
- **5C-11 连续曲率路线跟踪**：若实测证明 recovery lease/占用保护仍产生“有角速度、无平移”的 12 s 停滞，撤销该失败候选并恢复原 joint writer 生命周期。joint/direct 控制器对中等航向误差使用带最小 alignment factor 的低速弧线跟踪，只有接近反向的大角度才允许原地旋转；BRAKE、EMERGENCY、advance-grant、reservation expiry 与 peer safety shield 的零速语义完全不变。不得新增 planner 调用、IPC、device read 或逐帧日志；每步至少三轮 code review/test，并重新执行 Webots 性能门禁。
- **5C-12 自适应 rolling prefix 刷新**：普通 joint transaction 不再无条件按固定 2 s 覆盖仍有充足有效期的前缀；根据已提交 `waypoint_not_before_offsets` 的最晚 offset，在前缀耗尽前预留 1.5 s 准备窗口，并将普通刷新间隔限制在 2--6 s。prefix 完成、业务目标变化、成员变化或显式 safety/liveness request 必须立即 bypass；不得改变 scheduler、priority、候选评分、reservation、grant 或 validator。目标是降低 route dispatch/replan 与 epoch-boundary 转向损失，同时保持 Webots `sim_to_wall_ratio>=1.0`、P99<=50 ms、同 seed 吞吐退化不超过 10%。方案/workflow 各三轮 review，每个实现步骤至少三轮 code review/test。
- **5C-13 正面冲突 90° 侧向让行**：仅对经现有分类器确认的 head-on conflict，保持高优先级机器人业务路线、joint identity 和正常速度权限不变；低优先级机器人在相对冲突轴左右约 90° 的有限候选中选择通过 static segment、free grid、全部 peer clearance、winner predicted swept corridor 与可恢复业务路线验证的 lateral standoff，原子提交有限让行腿。到达 standoff 后持有带 winner/epoch 的 validated wait，待 winner 物理清空后恢复原业务目标。左右均无安全候选时回退现有 validated direct/joint/recovery，禁止强行侧移；BRAKE/EMERGENCY/stale pose/距离与 reservation validator 始终可覆盖“winner 继续前进”。候选只在 conflict 状态边沿计算并设置 cooldown，禁止逐 timestep 360° 搜索、重复 dispatch、全局冻结或新增 Webots I/O。方案和 workflow 各至少三轮 review，每个实现步骤至少三轮 code review/test，并重新执行 90/600/900 s Webots 活性、吞吐和倍率门禁。
- **5C-14 recovery 有效物理位移合同**：所有 physical recovery/standoff 最终目标相对下发时 fresh measured pose 必须至少 0.50 m，严格大于 controller 0.35 m waypoint arrival threshold 并保留跟踪余量；小于阈值的候选不得记为 recovery success 或刷新恢复 cooldown。该过滤统一接入 joint-stall、full-ring escape、reverse 与 priority lateral 候选，之后仍须通过原 grid/segment/peer/swept/static-suffix validator。无有效候选时沿用现有 fallback/retry，不得直接运动。检查为每候选 O(1)，禁止新增 planner、IPC、device read 或日志；每步至少三轮 code review/test，并执行 Webots 性能门禁。
- **5C-15 持续物理进展证据**：cross-epoch physical-progress lease 的 measured displacement 与 goal-improvement 阈值统一使用既有 `STALL_PROGRESS_DISTANCE=0.10 m`；0.05 m 级转向漂移、短暂向目标靠近后退回、route/epoch rewrite 均不得续租。保留原 5 s soft、8 s hard、recovery、priority 和 safety 规则。每步三轮 review/test，并重新执行 Webots 性能门禁。
- **5C-16 联合路线有界原地转向**：joint/direct 跟踪在接近反向时仍允许先原地对准，但同一连续大角度区间最多纯旋转 2.0 s；超过上限后必须沿当前已授权线段以不低于 12% alignment 的低速曲率继续对准，航向误差回到阈值内立即清除计时。该机制不得越过 advance grant、reservation deadline、BRAKE/EMERGENCY、stale pose 或 peer safety shield，也不修改 scheduler、priority、joint planner、候选评分和路径。实现前完成三轮方案 review：安全边界 `PASS`、状态/重置边界 `PASS`、O(1) Webots 性能 `PASS`；workflow 三轮 review：单元边界覆盖 `PASS`、90/600/900 s 递增门禁 `PASS`、失败即停止长测并保留证据 `PASS`。每个实现步骤至少三轮 code review/test；要求 0 个 >8 s 无计划停滞、`sim_to_wall_ratio>=1.0`、P99<=50 ms、同 seed 600 s 至少 14 tasks。
- **5C-17 active-prefix 刷新期限**：计算 ordinary rolling refresh deadline 时，只纳入尚未完成当前 transaction prefix 的成员；已消费完前缀、空 offset 或已脱离该 transaction 的成员不得把全车间隔永久压到 2 s。所有成员完成、目标/成员变化、显式 safety/liveness request 仍立即刷新，且 refresh 上限仍为 6 s，不改变 joint candidate、priority、reservation、grant 或 validator。方案三轮 review（活性 bypass、安全不变、O(n members) 性能）及 workflow 三轮 review（边界、回归顺序、失败停止）均须 `PASS` 后实施；每步至少三轮 code review/test，并重新执行 90/600/900 s 性能门禁。
- **5C-18 advance-grant 时空一致性**：对 fresh、同一 activated joint epoch 且身份完整的 peer，grant 复验必须信任已经通过统一 joint validator 的 `(cell,time_slot)` vertex/edge/residence 约束，不得再用 peer 当前点静态封锁申请机器人整条下一 segment；仍必须拒绝当前实测点对点小于 planning clearance、反向边、terminal owner、zone reservation、过期 reservation 及身份不匹配。stale pose、不同 epoch、direct/recovery/legacy route 继续沿用保守 swept-segment fail-safe。方案三轮 review 及 workflow 三轮 review 均通过后实施；每步至少三轮 code review/test，并执行 90/600/900 s Webots 门禁，报告 hold evidence、dispatch/replan、停滞、吞吐、倍率和 P99。
- **5C-19 连续时间 segment 占用复验**：替代失败的“同 epoch 直接放行”。仅对 fresh、双边 controller identity 与同一 activated joint epoch 完整匹配的 peer，以申请机器人当前点到目标点的有限执行时长、peer 实测位置/速度建立常速度连续时间模型，解析求 `[now, segment_end]` 内相对距离最小值；仅当同一时刻最小距离小于 planning clearance 时拒绝。速度未知/过低、时间窗无效、身份不完整、stale、跨 epoch、direct/recovery/legacy 一律沿用静态 swept fail-safe；reverse edge、当前硬间距、terminal/zone、reservation deadline 和动态 safety shield 不放松。热路径每 peer 仅常数次算术，禁止采样循环、planner、IPC、device read 和新增逐帧日志。方案/workflow 各三轮 review，每个实现步骤至少三轮 code review/test，并执行 90/600/900 s Webots 安全、吞吐、停滞、dispatch/replan、倍率及 P99 门禁。

#### Step 5C-10 Webots 性能硬约束

- recovery 占用必须按时间槽随物理速度前移，禁止把整条剩余路线在全部时隙静态封锁；禁止因一台 recovery robot 暂停全车队 rolling refresh。
- 热路径只能复用已有 measured pose、剩余 waypoints 和静态 grid；不得新增 Webots device read、同步 IPC、逐帧磁盘/事件日志或无界全图扫描。
- 90/600/900 s 均须报告 wall seconds、`sim_to_wall_ratio`、调度 P99、joint planning cost、route dispatch、最差单机器人 motion continuity 和逐机器人 unplanned stop；倍率必须 `>=1.0`、P99 `<=50 ms`。
- 相同 Scene C/FCFS/seed 的吞吐及完成率相对 Step 5B 通过基线不得退化超过 10%。画面出现多机器人无物理 block 停车、任何单机器人无计划停滞 `>8 s`，即使全局平均值通过也必须立即停止长测并返工。

#### Step 5C 硬门禁

- `inbound_grant_while_occupied=0`、`terminal_release_before_physical_clear=0`、`dual_terminal_owner=0`、`handoff_identity_violation=0`、未授权运动=0、距离违规=0、nonphysical recovery=0。
- 服务完成且无已验证物理阻塞时，清场 command-zero episode 仍必须 `<=8 s`；有真实冲突时必须记录 peer/resource/epoch 与 validator evidence，不能归为合法的无期限等待。
- 每次 handoff 必须满足 event—summary 对账：一次 owner acquire 至多对应一次 release；一次 release 至多签发一个当前 epoch inbound grant；stale/duplicate grant 不得刷新 owner 或等待期限。
- 保留 Step 5A/5B 连续运动门禁、`sim_to_wall_ratio >=1.0`、调度 P99 `<=50 ms`；相对相同 seed 的 Step 5B 通过基线，吞吐/完成率不得退化超过 10%。
- 发布结论必须逐 terminal 类型和逐 seed 报告清场延迟、handoff 等待、恢复次数、最差单机器人连续运动指标与充电排队，不得只用全局均值掩盖局部饥饿。

#### Step 5C Webots 性能保障与返工门禁

- terminal owner/clearance 更新必须复用每 timestep 已读取的 pose，不得新增 Webots device read、同步 IPC 或磁盘访问；grant 热路径只检查当前业务 terminal 与当前清场 terminal，禁止遍历全部 terminal × 全部机器人。
- terminal 事件只能在 `service_complete/physically_clear/inbound_denied/inbound_granted` 的状态边沿记录；相同 terminal/owner/epoch 的重复 deny/grant 必须去重，禁止逐状态包写事件。
- 30 s smoke 用于启动和功能审计，其启动/退出墙钟单独报告；稳态硬门禁从 90 s 开始，要求 `sim_to_wall_ratio >=1.0`、调度 P99 `<=50 ms`。90 s 未通过时立即停止 600/900/1800 s。
- 性能失败必须在相同机器、相同 Webots `--batch --no-rendering --mode=fast`、相同场景/scheduler/seed 下做 Step 5B 基线与 Step 5C 候选 A/B；若两者均低于 1.0，记录为环境性能阻塞，不得错误归因给代码，也不得发布；若只有候选失败，则回滚最近子步骤并定位 hot path。
- 通过 A/B 后仍须完成 600 s 与 900 s，报告 Webots wall seconds、仿真倍率、joint candidate 规划耗时、grant 检查次数、terminal 事件数和调度 P99；最终 10×1800 s 的最低倍率必须 `>=1.0`。

当前状态：`IN_PROGRESS / STEP 5C-10 IMPLEMENTATION`。Step 5C-9 的 600 s 实测仍复现 recovery route 被普通 joint rolling route 覆盖所造成的无计划停滞，失败样本保留且不得作为发布证据。Step 5B 的完成结论不被覆盖，Step 5C 只有 5C-10 及全部最终耐久门禁通过后才能标记完成。
