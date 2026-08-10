# PPO 在 Webots 中表现较差的原因分析与修复

## 1. 现象

现有 1800 秒 Webots 点估计中，PPO 在 Scene B/C 的平均任务等待时间较低，但最终完成吞吐量明显低于部分传统算法：

- Scene B：PPO_RL 约 1.57 tasks/min，SA 约 2.80 tasks/min；
- Scene C：PPO_RL 约 2.77 tasks/min，Hungarian 约 5.20 tasks/min；
- Scene C 中 PPO 平均等待时间约 76.11 秒，为当前记录中的最低值。

这说明 PPO 能较快分配任务，但分配出的任务未能以同样高的效率完成。问题主要发生在“选择机器人—任务组合”和“Webots 物理执行”之间，而不是简单的推理速度问题。

## 2. 主要原因

### 2.1 PPO 动作空间只选择机器人

当前 PPO 的动作维度为 `MAX_ROBOTS=8`：

```text
action = robot_id - 1
```

运行时任务并不是 PPO 选择的，而是固定采用：

```python
sorted_tasks = sorted(
    pending_tasks,
    key=lambda t: (-t.priority, t.arrival_time))
task = sorted_tasks[0]
```

因此 PPO 实际解决的是：

```text
为代码预先选定的最高优先级任务选择一台机器人
```

DQN 和 SARSA 使用固定机器人槽与任务槽组合，动作直接表示机器人—任务对。因此，它们能够学习“哪台机器人执行哪个任务”，表达能力强于当前 PPO。

### 2.2 PPO 看不到当前被选任务的位置

PPO 的状态包括：

- 每台机器人的位置、朝向、状态、电量和是否有任务；
- 待处理任务数量；
- 所有待处理任务到最近机器人的平均距离；
- 降采样拥堵图。

但是状态中没有编码当前最高优先级任务的：

- pickup 坐标；
- delivery 坐标；
- pickup→delivery 距离；
- 每个空闲机器人到该任务的真实路线成本。

这形成了结构性部分可观测问题：策略被要求选择一台机器人，但无法知道当前任务具体在哪里。

### 2.3 训练环境与 Webots 执行存在分布差异

PPO 训练使用独立的简化运动循环：

- 100 ms 训练步长；
- 理想化直线/航点移动；
- 不包含完整 LiDAR 和 DWA 动力学；
- 不包含真实 Webots 加减速、旋转和局部避让延迟；
- 不完整复现主动冲突扫描、紧急制动和物理停滞。

因此，抽象环境中看起来合理的机器人选择，在 Webots 中可能产生更长的实际执行时间。

### 2.4 PPO 拥堵惩罚在训练调用中没有获得真实事件

PPO 训练中的奖励调用是：

```python
scheduler.compute_reward(robot_states, 0, 0, 0.0)
```

其中 `congestion_events=0`，因此配置中的拥堵惩罚在该决策时刻没有根据真实拥堵事件生效。虽然状态包含拥堵图，但策略缺少稳定的直接拥堵奖励信号。

### 2.5 PPO 和 DQN/SARSA 使用了不同的训练环境与奖励体系

DQN/SARSA 使用 `SchedulingEnvironment`：

- 机器人—任务组合动作；
- 明确动作掩码；
- candidate-specific 特征；
- valid assignment、priority、age、waiting 和 distance 奖励。

PPO 使用旧的专用训练循环和另一组奖励常量：

- task completion；
- idle penalty；
- congestion penalty；
- empty travel distance；
- balance bonus。

所以三种 RL 算法的训练条件并不完全同构，不能仅凭 episode 数量判断 PPO 应当更强。

### 2.6 固定机器人 ID 动作缺少置换不变性

PPO 输出直接对应 robot 1～8。相同几何关系如果由不同机器人 ID 占据，网络需要分别学习。它可能形成对特定 ID 的偏好，而不是学习可泛化的相对路线关系。

## 3. 已实施的即时修复

### 3.1 Route-aware PPO reranking

在 `RLScheduler.assign()` 中增加了运行时路线感知层：

1. PPO 输出每台空闲机器人的策略概率；
2. 从高优先级/较老任务中选取有限候选；
3. 使用当前 `SchedulingContext.path_cost_provider` 计算真实路线成本；
4. 把 PPO 策略概率作为学习先验；
5. 将策略成本和路线成本归一化后联合排序；
6. 选择最低联合成本的合法机器人—任务组合。

联合得分为：

\[
S(i,j)=w_r\hat C_{route}(i,j)+(1-w_r)\hat C_{policy}(i)
\]

其中：

\[
C_{policy}(i)=-\log P_{PPO}(i|s)
\]

默认设置：

```text
PPO_ROUTE_WEIGHT=0.50
PPO_TASK_CANDIDATES=8
PPO_ROUTE_AWARE=1
```

即 PPO 策略和实时路线成本各占 50%。

这不是声称旧模型已经学会机器人—任务配对，而是一个透明的部署安全/效率修正层。

### 3.2 可关闭设计

为了支持消融对比，可设置：

```powershell
$env:PPO_ROUTE_AWARE = "0"
```

恢复原始 PPO 行为。

启用修复：

```powershell
$env:PPO_ROUTE_AWARE = "1"
$env:PPO_ROUTE_WEIGHT = "0.50"
$env:PPO_TASK_CANDIDATES = "8"
```

### 3.3 模型失败回退

PPO 模型缺失或不兼容时，回退算法已由 FCFS 调整为 Hungarian。这样与 DQN/SARSA 的安全回退原则更加一致，也避免模型加载失败被误认为真实 PPO 性能。

## 4. 短时预验证结果

使用相同 PPO checkpoint、Scene C、300 秒 standalone 闭环和相同 seed，比较原始 PPO 与 route-aware PPO。

### Seed 42

| 方法 | 完成任务 | 吞吐量 | 平均完成时间 | 平均等待时间 | 距离 |
|---|---:|---:|---:|---:|---:|
| 原始 PPO | 24/39 | 4.80 tasks/min | 85.30 s | 18.64 s | 462.4 m |
| Route-aware PPO，w=0.50 | 30/39 | 6.00 tasks/min | 78.50 s | 17.77 s | 462.7 m |

### Seed 43

| 方法 | 完成任务 | 吞吐量 | 平均完成时间 | 平均等待时间 | 距离 |
|---|---:|---:|---:|---:|---:|
| 原始 PPO | 18/28 | 3.60 tasks/min | 80.13 s | 1.21 s | 411.1 m |
| Route-aware PPO，w=0.50 | 20/28 | 4.00 tasks/min | 65.60 s | 0.52 s | 373.1 m |

两组平均：

- 完成任务数：21 → 25，约提高 19%；
- 平均完成时间：约 82.7 s → 72.1 s；
- 平均等待时间：约 9.93 s → 9.15 s；
- 平均距离：约 436.8 m → 417.9 m。

这些结果只用于快速回归验证，不是正式 Webots 统计结论。

## 5. 推荐正式验证

时间受限时可以只运行：

```text
Scene C
原始 PPO vs Route-aware PPO
seed 42、43、44
每次 600 秒
```

对比：

- throughput；
- average completion time；
- average waiting time；
- total distance；
- workload balance CV；
- pair-distance violations；
- scheduling latency P95。

这只需要 6 次运行，即可判断修复是否在 Webots 物理环境中稳定有效。

## 6. 长期正确方案

Route-aware reranking 是兼容旧 checkpoint 的快速修复。长期应将 PPO 重构为与 DQN/SARSA 相同的环境语义：

### 6.1 机器人—任务对动作

```text
action = (robot_slot, task_slot)
```

并包含 no-op 动作。

### 6.2 Candidate-specific 状态

至少编码：

- robot→pickup 路线距离；
- pickup→delivery 路线距离；
- 机器人电量；
- 任务等待时间和优先级；
- 路径拥堵；
- 预计充电风险；
- 当前任务槽是否有效。

### 6.3 与 DQN/SARSA 共用 SchedulingEnvironment

三种算法应共用：

- 相同 observation；
- 相同 action mask；
- 相同任务流；
- 相同奖励函数；
- 相同验证 seed；
- 相同模型选择指标。

这样才能把差异主要归因于 SARSA、DQN、PPO 的学习方法，而不是环境接口不同。

### 6.4 更接近 Webots 的训练随机化

训练环境应随机化：

- 行驶速度；
- 转向和加减速延迟；
- 路径执行时间；
- 局部避障等待；
- 充电时间；
- 路径失败和短时阻塞。

这可以减小 abstract-to-Webots distribution gap。

## 7. 结论

PPO 表现较差的首要原因不是 PPO 算法本身，而是当前 PPO 接口存在动作和观察不匹配：策略只选择机器人，却看不到被固定选择任务的具体位置。DQN/SARSA 的机器人—任务组合动作更符合实际调度问题。

当前 route-aware reranking 在两个短时相同-seed 预验证中提高了平均完成任务数并降低了平均完成时间。正式结论仍应通过 3 个 seed 的短时 Webots A/B 测试确认。长期应重新训练 pair-action PPO，并使三种 RL 方法共享同一调度环境和奖励体系。

## 8. 后续实现状态更新

上述长期方案现已实现：新 PPO 使用与 DQN/SARSA 相同的 `SchedulingEnvironment`、278 维 observation、161 维机器人—任务联合动作及一致的 action mask。PPO buffer 会保存每一步 mask，更新阶段用相同 mask 计算新旧 log probability。旧 126×8 checkpoint 仅作为兼容路径保留。

详细迁移说明见 `docs/PPO_PAIRWISE_TRAINING_MIGRATION_CN.md`。新模型仍需重新进行完整训练和 Webots 多 seed 评估后，才能判断实际差距是否消除。
