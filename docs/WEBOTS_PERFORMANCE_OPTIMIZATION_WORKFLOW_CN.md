<!--
  Webots 性能优化 Workflow
  适用场景：在不改变现有调度、规划、安全、事务等基本业务逻辑的前提下，
  修复机器人运行卡顿并保证 Webots 模拟性能。
-->

# Webots 性能优化 Workflow

状态：`ACTIVE`

适用范围：`smart_factory_scheduler` 仓库中所有涉及 Webots 运行时性能、
机器人控制器、Supervisor、联合规划器及世界配置的性能优化步骤。

## 1. 目标

- **最高目标：提高机器人运行效率，防止机器人卡顿、走一步停一下。**
- 降低 Webots 中机器人运行卡顿和瞬时冻结。
- 保证 Webots 模拟性能不回退。
- 不改变现有调度策略、路径规划语义、安全距离阈值、任务优先级、
  联合规划事务协议和回滚规则。
- 每一小步都必须留下可复现的代码审查记录、Webots 录屏证据和性能指标。
- 评价一个优化是否有效，优先看机器人运动连续性，而不是只看墙钟或
  规划器单项耗时。

## 2. 每小步强制流程

每一个独立修改步骤必须按以下顺序完成，未完成不得进入下一步：

1. **冻结基线**：记录当前代码状态、运行命令、环境变量、Webots 版本、
   场景、种子、持续时间和结果目录。
2. **单变量修改**：一步只改一个性能变量；禁止把多个优化混在一个步骤里。
3. **本地验证**：先跑针对该修改的最小验证和必要的 pytest。
4. **Code Review R1**：检查改动范围、语义等价性和依赖影响。
5. **Code Review R2**：检查安全、状态机、并发/事务、性能、回滚与可测试性。
6. **Webots 录屏验证**：运行修改后的场景并生成 Webots 动画录屏和结果 JSON。
7. **性能门禁**：与上一步基线对比，满足所有性能和安全指标。
8. **记录结论**：把 R1、R2、录屏路径、指标和结论写入本步骤验收记录。

### 2.1 Code Review 强制要求

- 每一步至少完成两次独立 code review，分别记作 `R1` 和 `R2`。
- `R1` 必须回答：
  - 修改是否只影响性能，不改变业务决策？
  - 输入相同的情况下，调度、路径、安全判断是否保持相同？
  - 是否有隐藏的 API、状态机或协议变化？
- `R2` 必须回答：
  - 是否引入新的同步阻塞、热路径 I/O、无限重试或锁竞争？
  - 是否影响机器人安全距离、碰撞避免、事务提交或回滚？
  - 是否便于回滚和复现？
- 两次 review 都通过才允许进入 Webots 录屏验证。
- 任一 review 发现高风险管理问题，该步骤状态为 `BLOCKED` 或 `FAILED`。

### 2.2 Webots 录屏验证要求

- 每个小步必须生成 Webots 动画录屏证据。
- 优先使用 Supervisor 已有的
  `SMART_FACTORY_DIAGNOSTIC_RECORDING=1` 机制。
- 如用户要求真实 GUI 录屏，另存屏幕录像文件。
- 录屏必须能观察到机器人运动是否卡顿、是否出现异常停车、路径是否稳定。
- 录屏文件统一写入 `results/performance_workflow/<step_id>/`。
- 每个 step 至少保留：
  - Webots 动画录屏（HTML/X3D 或视频）
  - 实验结果 JSON
  - stop analysis JSON（如适用）
  - 性能摘要 JSON/CSV

## 3. 性能门禁

### 3.0 机器人运行效率主门禁（最高优先级）

- 录屏中不得出现比上一步更多或更长的“走一步停一下”。
- `abnormal_mass_stop_seconds` 不得上升，优先下降。
- `unexplained_active_intervals` 不得上升。
- `rotating_in_place_intervals` 不得上升。
- `unplanned_task_stops` 不得上升。
- `motion_continuity.clear_motion_duty_cycle` 不得下降。
- 任一运行效率主指标退化，即使墙钟/规划耗时改善，该步骤也必须回滚或
  标记 `FAILED`。
### 3.1 Webots 模拟性能

- 自动化 fast/no-rendering 场景：
  - `sim_to_wall_ratio` 中位数不得低于上一步基线。
  - `sim_to_wall_ratio` 最差值不得低于 1.00。
- Supervisor 整步耗时：
  - `supervisor_step_wall_p50_ms` 不得明显上升。
  - `supervisor_step_wall_p99_ms` 不得明显上升。
  - `supervisor_step_wall_max_ms` 不得出现新的 500ms+ 冻结。
- 联合规划耗时：
  - `joint_planning_wall_p99_ms` 不得明显上升。
  - 规划失败不得造成新的密集重复触发。

### 3.2 安全与业务稳定性

- `min_pair_distance` 不得低于上一步基线，且不得低于安全阈值。
- `pair_distance_violations == 0`。
- `unplanned_task_stops` 不得增加。
- `route_dispatches`、`rapid_route_overrides`、`replan_events` 不得异常增长。
- 任务完成数、路径稳定性、charging/terminal 行为不得因性能修改退化。

### 3.3 录屏人工/自动复核

- 录屏中不得出现比上一步更多或更长的机器人停顿。
- 重点检查是否存在规律性“走一步停一下”，而不是只关注单次长停。
- 录屏中不得出现新的异常绕行、倒车、碰撞或穿越障碍。
- 如有条件，计算动画帧间隔或记录 GUI FPS，作为辅助指标。
- 当单次墙钟波动较大时，应增加同会话确认录屏；但运行效率主门禁仍必须
  满足，不能因墙钟噪声放宽。

## 4. 实施步骤

以下为当前计划，每个步骤内部都执行第 2 节流程：

### Step 0：基线冻结与录屏

- 不修改运行代码。
- 固定当前工作区状态，记录当前 dirty 文件清单。
- 运行当前场景并生成基线 Webots 录屏和结果 JSON。
- 建立基线指标：`sim_to_wall_ratio`、step wall、joint planning wall、
  安全距离、任务数、停车事件。

### Step 1：分阶段性能观测

- 只增加观测埋点，不改变决策。
- 分别记录：
  - `Supervisor.step()` 本身耗时
  - Python 各阶段耗时
  - joint planning 各层耗时
  - robot DWA 耗时
- 用录屏确认观测代码本身不增加明显卡顿。

### Step 2：机器人 DWA 等价加速

- 保持 DWA 评分函数、安全距离、采样范围和最终决策不变。
- 只做计算实现优化。
- 通过单元测试和 Webots 录屏确认机器人轨迹不回退。

### Step 3：联合规划器热点优化

- 保持规划语义、优先级顺序、冲突判定和事务协议不变。
- 只优化数据结构、临时对象和热循环实现。
- 用真实难例快照做前后耗时对比。

### Step 4：同步规划 burst 控制

- 为联合规划增加更严格 wall-time 上限或失败退避。
- 只调整触发节奏/预算，不改变规划算法和安全判定。
- 录屏重点观察是否出现新的 long stop 或任务完成率下降。

### Step 5：Webots 世界/运行配置

- 仅在代码优化仍不足时评估：
  - GUI 下渲染/背景简化
  - LiDAR 降采样
  - 基本时间步调整
- 每一项单独验证，任何安全或行为退化立即回滚。

### Step 6：回归验证

- 使用多 seed/多场景进行回归。
- 最终必须通过全部性能、安全和录屏门禁。

## 5. 回滚规则

- 每一步必须可独立回滚。
- 禁止 `git reset --hard` 或整文件覆盖式回滚。
- 若录屏或指标出现以下任一情况，立即回滚：
  - 性能门禁失败
  - 机器人运动出现新的明显卡顿
  - 安全距离或碰撞指标退化
  - 任务完成率/路径稳定性退化
- 回滚后重新跑录屏和指标，确认恢复到上一步基线。

## 6. 验收记录格式

每个 step 完成后创建：

```text
docs/performance_workflow_logs/<step_id>_<date>_<time>.md
```

记录内容：

- 目标与单变量说明
- 修改文件清单
- Code Review R1 结论
- Code Review R2 结论
- pytest / compileall / 静态检查结果
- Webots 录屏路径
- 性能指标前后对比
- 安全与业务指标前后对比
- 最终状态：`PASS` / `FAILED` / `BLOCKED`

## 7. 当前基线参考

最近一次 180s Scene C/FCFS/seed 44 基线结果：

- `sim_to_wall_ratio=1.467`
- `supervisor_step_wall_p50_ms=5.04`
- `supervisor_step_wall_p95_ms=9.39`
- `supervisor_step_wall_p99_ms=25.90`
- `supervisor_step_wall_max_ms=830.25`
- `joint_planning_wall_p95_ms=203.79`
- `joint_planning_wall_p99_ms=216.68`

这些数据只作为初始参考；每个 step 都必须重新录制自己的上一步基线。
