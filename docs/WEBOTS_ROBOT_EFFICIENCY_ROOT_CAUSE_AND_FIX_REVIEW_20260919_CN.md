# Webots 机器人效率根因复核与修复方案双重 Review

日期：2026-09-19  
范围：Supervisor 联合规划、事务切换、机器人联合路线跟踪、运行时 watchdog。  
本文件只给出方案和审查结论，不授权跳过逐小步 Webots 录屏门禁。

## 1. 复核后的根因排序

### P0：有效路线 epoch 更新过密，物理执行周期小于路线稳定周期

180 s 基线有 224 次 route dispatch，其中 209 次来自
`joint_grid_transaction`，另有 29 次 rapid override，但只完成 1 个任务。
`_refresh_joint_grid_candidate()` 已有“未启动成员保护”和候选生成后的三格
prefix 等价检查，但等价检查发生在昂贵规划之后，而且大量候选仍被判断为
materially different 并提交。每次激活都会令 Supervisor 和 controller 的
waypoint index 回到新 epoch 的 0，并重置分段进度统计。安全路径可能仍正确，
但物理机器人反复重新追踪一个略有变化的近端目标。

### P0：同步联合规划占用 Supervisor timestep

`plan_joint_grid_candidate()` 在 Supervisor 循环中同步执行，当前联合规划 P99
为 323--476 ms，而 Webots basic timestep 约 32 ms。这主要造成 GUI/墙钟卡顿，
也会延迟 controller status、事务 ACK 和下一轮命令处理。它不是动画仿真时间内
所有停车的直接原因，但会放大 epoch handoff 和遥测陈旧问题。

### P1：路线刷新节奏按事务时间估计，不按实测物理剩余进度

`_joint_refresh_interval()` 使用 waypoint offset 的末端时间决定 2--6 s 刷新，
而真实机器人会受到转向、物理滑移、speed scale 和安全 hold 影响。物理进度
落后时，新 epoch 可能过早覆盖仍可执行的 prefix；物理进度领先时，successor
又可能准备过晚，机器人到 `joint_window_endpoint` 后停车。

### P1：窗口端点/deadline 的零速是协议等待，不一定显示成 block

controller 在 `joint_window_endpoint`、`joint_epoch_barrier`、有效 advance hold、
reservation deadline 过期时返回 `(0, 0)`。其中安全 hold 必须保留；窗口端点和
过期 deadline 应通过提前准备 successor 降低，而不是取消零速保护。

### P1：频繁改变近端目标导致重复对向、低速曲率和倒车

联合跟踪在航向误差大于 1.20 rad 时允许最多 2 s 原地 pivot，之后才以有界
倒车/曲率继续。单独看该控制是有界且必要的；问题是新 epoch 改变首段方向时，
机器人会重新进入大角度修正。录屏有 32--35 段原地转向、约 86.24
robot-seconds turning。直接缩短 pivot 或全局提高线速度此前已导致任务效率退化，
因此不是首选修复。

### P2：规划失败后的 0.5 s 重试会形成 burst

没有 candidate，或 component 需要扩展为全 active fleet 时，会把下一次 tick
推进到 0.5 s 后。失败预算还会从 0.25 s 逐步增长到 1 s。旧 prefix 虽然多数
情况下继续执行，但连续同步重试会明显降低 Webots 墙钟性能，并可能让 route-less
成员等待较长。

## 2. 修复方案

### Step A：规划请求原因分级和无损合并（先实施）

把 pending request 从单一 robot-id set 改为带原因的请求表：

- `SAFETY`：预测冲突、emergency，最高优先级，立即 bypass 所有节流；
- `LIVENESS`：route-less、hard progress lease、陈旧 endpoint；
- `BUSINESS`：新任务、业务目标或成员变化；
- `ROLLING`：普通 prefix 补充，最低优先级。

请求只有在成功启动对应 successor transaction 后才消费。因 active transaction、
未启动成员或节流而 defer 时必须保留，防止当前函数入口清空
`_pending_joint_plan_robot_ids` 后丢失边沿事件。

验收：相同安全/业务输入的规划成员、优先级和最终目标不变；新增 request reason
计数；不得增加 route-less 首路线延迟。

### Step B：基于实测进度的 pre-plan eligibility gate（核心修复）

在调用联合规划器之前判断是否真的需要 successor：

- SAFETY/LIVENESS/BUSINESS 永远 bypass；
- ROLLING 只有在任一成员满足以下条件时才规划：
  - 剩余非重复 cell 少于 2；
  - controller index 对应的 reservation 剩余时间小于动态准备窗口；
  - 当前目标/成员集合改变；
  - 当前 prefix 已完成。
- 动态准备窗口取最近联合规划 P99 + prepare/arm/commit P99 + 0.5 s 裕量，
  并限制在 1.0--3.0 s，避免固定 2 s/6 s 对所有负载一刀切。

该 gate 只减少无收益规划，不改变 planner、冲突约束或安全距离。旧 prefix 在
successor 完成原子激活前继续执行。

验收：joint planning calls、route dispatch 和 rapid override 显著下降；first
dispatch 不变差；任务完成数不下降；异常低进度、原地转向和 duty 均不得退化。

### Step C：successor 的执行收益门（第二阶段）

扩展现有 `_joint_candidate_preserves_active_prefix()`：除前三格 cell 等价外，
增加“当前执行方向稳定性”判断。对于仍有至少两个可执行 cell 的成员，如果新
首个有效移动向量相对旧首段发生大角度翻转，且新方案没有解决 safety/liveness/
business 事件，则保留旧 epoch，延迟到准备窗口再次规划。

禁止按单机器人拆掉联合候选；判定应针对整个 transaction 原子接受或延后。
禁止把路径总长度更短作为唯一收益，因为较短路径仍可能造成近端 180° 转向。

验收：`rotating_in_place_intervals` 和 `_command_reverse` 不上升，任务数不下降，
安全距离不下降。

### Step D：联合规划分时化，消除 Supervisor 长暂停（独立实施）

将联合搜索改为基于纯 Python/不可变快照的增量 planning job：

- Webots API、Robot node 和 emitter 只能留在 Supervisor 主线程；
- 每个 timestep 最多给 planning job 8--12 ms wall budget；
- 每个 job 固定 generation、pose/goal/epoch snapshot；输入变化时丢弃 job；
- 旧 committed prefix 在 job 完成和事务激活前保持权威；
- safety emergency 不等待普通 job，继续走现有立即制动和 recovery 路径；
- 禁止用后台线程直接访问 Webots 对象。

验收：Supervisor P99 目标 <=50 ms，Webots 无录屏倍率 >=1.0；规划结果必须与
相同输入的一次性求解满足相同 validator，不能以超时接受 partial unsafe plan。

### Step E：仅在 A--D 通过后评估跟踪器

如果路线 churn 已下降但录屏仍有长原地转向，再对“同一稳定 target 持续 pivot”
单独处理。可选方案是保留 2 s 上限，在 0.6--1.2 rad 区间允许有界正向曲率；
大于 1.2 rad、倒车目标、safety hold、deadline 和 terminal 区域仍保持现逻辑。
不得全局提高速度、自动跳 waypoint 或取消 reservation deadline。

## 3. 修复方案 Review 1：正确性与根因匹配

结论：`PASS_WITH_ORDER_CONSTRAINTS`。

1. Step A 解决请求被合并/延后时缺乏 provenance 的问题，并为后续 bypass 提供
   明确语义；不改变 scheduler 或规划优先级。
2. Step B 在规划前消除无收益工作，直接针对 209 次 joint dispatch 和同步 P99；
   比单纯延长固定 refresh interval 更符合实测进度。此前固定 3 s 实验没有触及
   liveness/conflict 触发，因此失败；本方案按原因分级，不重复该实验。
3. Step C 直接针对重复对向，但必须在 A/B 后实施，否则高频安全/liveness 请求
   会让方向门频繁 bypass，无法判断收益。
4. Step D 解决 Webots 墙钟卡顿，和机器人行为修复正交；旧路线持续执行保证
   分时计算不会主动停车。
5. Step E 风险最高且不是首因，只能最后实施。以前全局曲率和 speed-scale 实验
   已证明局部运动指标改善可能伴随任务数下降。

Review 1 驳回的方案：直接提高轮速、缩短所有 pivot、取消 joint wait、自动跳过
近点 waypoint、只扩大 planner timeout、只减少 planner 调用但不检查任务完成数。

## 4. 修复方案 Review 2：安全、状态机、性能与回滚

结论：`PASS_WITH_MANDATORY_GATES`。

### 安全

- SAFETY 请求必须绕过 Step B/C；现有 dynamic shield、radial emergency stop、
  reservation validator 和原子事务均不得删除。
- 延用旧 prefix 只允许在其 epoch、deadline、goal 和成员仍有效时；deadline
  过期必须停车，不能为了连续运动越过未验证 cell。
- Step C 必须以整个 transaction 为单位决定，不允许一部分机器人切新 epoch、
  另一部分留旧 epoch。

### 状态机

- pending 请求不得在 defer、planner timeout 或 transaction prepare 失败时消费；
  只有 successor transaction 成功进入 preparing 后才标记 claimed，activated 后
  才最终完成。
- 新请求到来时允许提升原因等级，禁止由 ROLLING 覆盖 SAFETY/LIVENESS。
- generation snapshot 必须包含 robot ids、实测 pose、business goal、active epoch、
  path version 和 terminal ownership。

### 性能

- Step A/B/C 的 precheck 必须为 O(机器人数量 × 有界 prefix)，禁止扫描整条历史
  路线或在 timestep 写文件。
- Step D 每步预算有硬上限；job 未完成时直接让 Webots 进入下一 timestep。
- 录屏行为跑和无录屏性能跑必须分离，不能用动画导出后的倍率判断模拟性能。

### 回滚和实施顺序

每步独立提交/补丁，顺序必须是 A → B → C → D → 可选 E。每一步执行：专项测试、
组合测试、全量测试、180 s 同 seed Webots 录屏、同配置无录屏性能跑；失败立即只
回滚该步。不得把多项控制改变塞进一次录屏。

### 硬门禁

- tasks completed 不低于上一通过基线；
- abnormal mass stop、unexplained active、rotating in place、unplanned stops
  均不得上升；
- clear-motion duty 不得下降；
- pair-distance violations = 0，最小距离不得下降；
- fast/no-rendering `sim_to_wall_ratio >= 1.0`；
- Supervisor P99 和 joint planning P99 不得恶化，Step D 目标 Supervisor P99
  <=50 ms。

## 5. 最终建议

首个实现小步只做 Step A；第二小步做 Step B。不要先改机器人速度控制。若 A/B
使 dispatch/replan/规划 P99 下降，但 tasks、低进度或转向不改善，则判失败并回滚，
转而从 terminal handoff 和具体未解释区间继续做录屏因果分析。
