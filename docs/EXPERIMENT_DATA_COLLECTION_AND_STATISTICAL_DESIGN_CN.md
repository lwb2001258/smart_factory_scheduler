# 智能工厂多机器人实验数据采集与统计分析设计

## 1. 文档目的

当前 Webots 实验已经覆盖 Scene A、Scene B、Scene C 以及多种传统、优化和强化学习调度算法，但大部分“算法—场景”组合只有一次完整运行，并且现有结果 JSON 没有记录随机种子。因此，目前结果只能作为点估计，无法可靠计算跨随机种子的标准差、置信区间或统计显著性。

本设计用于建立一套可重复、可追溯的数据管线：

```text
实验配置表
   ↓
批量运行 A/B/C × 调度算法 × 随机种子
   ↓
每次运行输出独立 JSON
   ↓
校验运行完整性、随机种子和配置一致性
   ↓
汇总为 run-level CSV
   ↓
计算均值、标准差和 95% 置信区间
   ↓
生成带误差棒的论文及答辩图表
```

## 2. 推荐实验矩阵

第一阶段建议使用：

- 场景：A、B、C；
- 核心算法：FCFS、NearestNeighbour、Hungarian、Auction、GA、SA、SARSA、DQN、PPO_RL；
- 随机种子：42、43、44、45、46；
- 单次仿真时间：1800 秒；
- 每个“算法—场景”组合至少运行 5 次。

核心实验总数为：

\[
3\text{ 个场景}\times9\text{ 个算法}\times5\text{ 个种子}=135\text{ 次}
\]

如果加入 Greedy、RoundRobin 和 Random，则实验总数为：

\[
3\times12\times5=180\text{ 次}
\]

答辩前应优先完成 9 个核心算法的 135 次实验。

## 3. 实验元数据与随机种子

### 3.1 结果 JSON 必须记录 seed

建议将每次运行的基本信息保存为：

```json
{
  "experiment_info": {
    "scenario": "C",
    "scheduler": "Hungarian",
    "seed": 42,
    "num_robots": 8,
    "sim_duration": 1800.0,
    "completed_normally": true,
    "world_file": "worlds/smart_factory.wbt",
    "timestamp": "2026-08-04 12:00:00"
  }
}
```

建议同时保存：

- Git commit hash；
- Webots 版本；
- 配置文件版本或 SHA256；
- RL 模型路径和模型 SHA256；
- 是否启用 RHCR；
- 是否启用 legacy interlock recovery；
- GA、SA 的实际运行参数；
- RL checkpoint 类型和环境版本。

### 3.2 统一随机性

同一个 seed 应控制：

- 任务到达时间；
- 任务起点和终点；
- 任务优先级；
- 机器人初始电量；
- Random 调度器；
- GA 初始种群、交叉和变异；
- SA 邻域生成和接受过程；
- RL 推理随机性。

RL 正式评估时通常应关闭探索，使用冻结模型的确定性或固定随机种子策略。

### 3.3 固定任务流

为了保证算法间公平，最好在正式实验前生成任务流文件：

```json
{
  "scenario": "C",
  "seed": 42,
  "tasks": [
    {
      "arrival_time": 0.0,
      "pickup": "S3",
      "delivery": "WS2",
      "priority": 1.21
    }
  ]
}
```

相同场景和 seed 下的所有调度算法读取同一个任务流，而不是各自重新随机生成。这样可以形成配对实验，也便于审计任务序列是否完全一致。

## 4. 单次运行应保存的数据

### 4.1 任务级数据

每个任务建议保存：

```json
{
  "task_id": 25,
  "arrival_time": 318.4,
  "assignment_time": 327.1,
  "pickup_time": 365.7,
  "completion_time": 442.6,
  "waiting_time": 8.7,
  "pickup_travel_time": 38.6,
  "delivery_travel_time": 76.9,
  "completion_duration": 124.2,
  "assigned_robot": 4,
  "priority": 1.32,
  "status": "completed",
  "reassignment_count": 0
}
```

由此可以计算：

- 任务等待时间；
- 取货行驶时间；
- 送货行驶时间；
- 总执行时间；
- 总完成时间；
- 完成率；
- 高优先级任务服务质量；
- 任务重新分配次数。

任务时间关系为：

\[
T_{waiting}=T_{assignment}-T_{arrival}
\]

\[
T_{execution}=T_{completion}-T_{assignment}
\]

\[
T_{completion}=T_{waiting}+T_{execution}
\]

### 4.2 机器人级数据

每台机器人建议累计：

```json
{
  "robot_id": 4,
  "productive_time_s": 1042.3,
  "empty_travel_time_s": 311.8,
  "loaded_travel_time_s": 730.5,
  "idle_available_time_s": 425.2,
  "charging_time_s": 270.4,
  "waiting_for_path_time_s": 61.3,
  "blocked_time_s": 43.6,
  "distance_empty_m": 82.5,
  "distance_loaded_m": 164.7,
  "energy_consumed_pct": 73.2,
  "tasks_completed": 12
}
```

需要区分：

- 携货行驶；
- 空载前往取货点；
- 前往休息点；
- 空闲等待；
- 冲突阻塞；
- 等待重规划；
- 充电；
- 前往充电站。

### 4.3 Productive utilisation

建议将生产性利用率定义为：

\[
U_{productive}=\frac{T_{loaded}+T_{pickup/service}}{T_{simulation}}\times100\%
\]

同时保存活动利用率：

\[
U_{active}=\frac{T_{empty\ travel}+T_{loaded\ travel}+T_{service}}
{T_{simulation}-T_{charging}}\times100\%
\]

这样可以区分真正创造运输价值的时间和仅仅处于运动状态的时间。当前 `avg_robot_idle_pct` 接近零，可能是因为前往休息点等非生产运动仍被统计为 active，因而区分能力有限。

## 5. GA 优化轨迹

当前 GA 结果只包含最终分配、执行代数和计算时间，没有保存逐代 fitness。

### 5.1 建议数据结构

```json
{
  "scheduling_event_id": 87,
  "sim_time": 412.8,
  "algorithm": "GA",
  "population_size": 48,
  "generation_limit": 50,
  "time_budget_ms": 10,
  "generations_executed": 34,
  "trace": [
    {
      "generation": 0,
      "best_cost": 42.63,
      "mean_cost": 51.27,
      "feasible_ratio": 0.83,
      "elapsed_ms": 0.24
    },
    {
      "generation": 1,
      "best_cost": 39.81,
      "mean_cost": 47.12,
      "feasible_ratio": 0.91,
      "elapsed_ms": 0.46
    }
  ],
  "final_best_cost": 31.46,
  "improvement_pct": 26.20,
  "stop_reason": "time_budget"
}
```

每代至少记录：

- best cost；
- mean cost；
- 可行解比例；
- 实际种群大小；
- elapsed time；
- 停止原因。

### 5.2 GA 收敛曲线

由于不同调度时刻的问题规模不同，不应直接平均原始 cost。建议计算归一化成本：

\[
C_{relative,g}=\frac{C_g}{C_0}
\]

或改进百分比：

\[
I_g=\frac{C_0-C_g}{C_0}\times100\%
\]

图表可使用：

- 横轴：Generation；
- 纵轴：Mean normalised best cost；
- 阴影：跨实验运行的 95% CI。

## 6. SA 优化轨迹

### 6.1 建议数据结构

```json
{
  "scheduling_event_id": 92,
  "initial_temperature": 10.0,
  "cooling_rate": 0.985,
  "minimum_temperature": 0.01,
  "iteration_limit": 2000,
  "time_budget_ms": 5,
  "iterations_executed": 627,
  "trace": [
    {
      "iteration": 0,
      "temperature": 10.0,
      "current_cost": 43.8,
      "best_cost": 43.8,
      "delta_cost": 0.0,
      "accepted": true,
      "elapsed_ms": 0.02
    }
  ],
  "accepted_better": 37,
  "accepted_worse": 12,
  "rejected": 578,
  "stop_reason": "time_budget"
}
```

每次迭代建议记录：

- temperature；
- current cost；
- best cost；
- delta cost；
- 是否接受；
- 接受原因；
- elapsed time。

SA 接受劣解的概率为：

\[
P=\exp\left(-\frac{\Delta E}{T}\right)
\]

为了控制 JSON 大小，可以采用抽样记录：

- 前 100 次迭代逐次记录；
- 之后每 10 次记录一次；
- 最终迭代强制记录。

可生成以下图表：

- iteration vs current cost；
- iteration vs best cost；
- iteration vs temperature；
- 接受劣解比例。

## 7. 安全、冲突与拥堵事件

项目当前同时包含：

- `conflicts_detected`；
- `conflicts_resolved`；
- `pair_distance_violations`；
- `min_pair_distance`；
- `deadlocks`；
- `replans`；
- 紧急制动；
- 重规划请求；
- 安全迁移。

这些指标应使用统一事件结构记录。

### 7.1 安全事件格式

```json
{
  "event_id": 143,
  "sim_time": 729.42,
  "type": "separation_violation",
  "severity": "warning",
  "robot_ids": [2, 7],
  "distance_m": 0.43,
  "relative_speed_mps": 0.18,
  "ttc_s": 1.36,
  "location": [0.12, 1.48],
  "resolution": "robot_7_hold",
  "resolved": true,
  "duration_s": 2.14
}
```

### 7.2 推荐事件类型

- `predicted_path_conflict`；
- `vertex_reservation_conflict`；
- `edge_swap_conflict`；
- `separation_warning`；
- `separation_violation`；
- `emergency_braking`；
- `replan_requested`；
- `replan_completed`；
- `deadlock_detected`；
- `safe_relocation`；
- `physical_collision`。

### 7.3 阈值定义

阈值应集中配置并写入结果元数据，例如：

- 警告距离：小于 0.7 m；
- 安全间距违反：小于 0.5 m；
- 真实碰撞：Webots 接触传感器触发或碰撞包围体相交；
- 死锁：机器人连续一定时间没有有效进展，并形成相互阻塞关系。

“预测冲突”“安全间距违反”和“物理碰撞”必须分别统计。

### 7.4 避免重复计数

如果两台机器人连续 2 秒都低于安全距离，不能按照每个仿真步分别计数。应使用事件状态机：

```text
SAFE
  ↓ 低于阈值
VIOLATION_STARTED
  ↓ 持续低于阈值
VIOLATION_ACTIVE
  ↓ 恢复安全距离
VIOLATION_ENDED
```

一次持续事件只记录一次，同时保存：

- 开始时间；
- 结束时间；
- 持续时间；
- 最小距离；
- 最大相对速度；
- 最小 TTC。

## 8. 统计分析方法

### 8.1 描述统计

对每个“场景—算法—指标”组合计算：

- 样本量 `n`；
- mean；
- standard deviation；
- minimum；
- maximum；
- median；
- 95% confidence interval。

建议聚合 CSV 结构：

| Scenario | Algorithm | Metric | N | Mean | SD | Min | Max | CI Low | CI High |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|
| C | PPO | Waiting Time | 5 | 78.4 | 6.2 | 70.1 | 86.5 | 70.7 | 86.1 |

样本量较小时，95% CI 使用 Student's t 分布：

\[
\bar{x}\pm t_{0.975,n-1}\frac{s}{\sqrt{n}}
\]

当 `n=5` 时不应直接使用固定的 1.96。

### 8.2 配对统计检验

如果同一个 seed 下所有算法使用相同任务流，可以采用配对分析：

- 两种算法：paired t-test；
- 非正态两算法：Wilcoxon signed-rank test；
- 多算法：repeated-measures ANOVA；
- 非正态多算法：Friedman test；
- 多重比较：Holm correction。

除了 p-value，还应报告：

- paired Cohen's \(d\)；
- Wilcoxon effect size \(r\)；
- 相对改进百分比。

## 9. RL 消融实验

### 9.1 Reward ablation

建议先在高负载 Scene C 中对最佳 RL 模型进行消融：

| 实验 | 奖励设置 |
|---|---|
| Full | 完整奖励函数 |
| No distance | 删除距离惩罚 |
| No waiting | 删除等待时间惩罚 |
| No priority | 删除优先级奖励 |
| No age bonus | 删除任务年龄奖励 |
| Weak collision | 减小碰撞惩罚，仅用于仿真安全分析 |

每个消融设置至少使用 5 个相同评估 seed，比较：

- throughput；
- waiting time；
- completion time；
- travel distance；
- separation violations；
- productive utilisation。

### 9.2 Coordination ablation

保持调度器不变，分别测试：

| 配置 | 协调功能 |
|---|---|
| Full | 栅格预留、同伴预测、DWA、重规划和恢复全部启用 |
| No reservation | 移除时空路径预留 |
| No peer prediction | 移除机器人同伴轨迹预测 |
| No proactive scan | 移除 Supervisor 主动冲突扫描 |
| No relocation | 移除仿真安全迁移 |

可能造成明显碰撞的消融只能用于仿真环境。

### 9.3 分离调度与协调贡献

为了回答性能提升来自调度器还是路径协调层，建议进行二维实验：

```text
同一调度器 + 不同协调层
不同调度器 + 同一协调层
```

这样可以分别估计：

- 调度算法贡献；
- 路径协调贡献；
- 二者交互贡献。

## 10. 推荐结果目录

```text
results/
└── dissertation_runs/
    ├── manifest.json
    ├── task_streams/
    │   ├── A_seed42.json
    │   └── ...
    ├── raw/
    │   ├── A/
    │   │   ├── FCFS/
    │   │   │   ├── seed_42.json
    │   │   │   └── ...
    │   │   └── PPO_RL/
    │   └── C/
    ├── optimisation_traces/
    │   ├── ga/
    │   └── sa/
    ├── safety_events/
    ├── summary/
    │   ├── run_level.csv
    │   ├── aggregate_metrics.csv
    │   └── hypothesis_tests.csv
    └── figures/
```

`manifest.json` 建议保存完整实验矩阵和当前状态：

```json
{
  "scenarios": ["A", "B", "C"],
  "algorithms": [
    "FCFS", "NearestNeighbour", "Hungarian", "Auction",
    "GA", "SA", "SARSA", "DQN", "PPO_RL"
  ],
  "seeds": [42, 43, 44, 45, 46],
  "duration_s": 1800,
  "expected_runs": 135,
  "completed_runs": 0
}
```

## 11. 实施顺序

1. 在结果 JSON 中加入 seed、模型、代码版本和配置元数据。
2. 预生成固定任务流，保证算法间公平。
3. 增加机器人分状态时间和分类型距离统计。
4. 统一安全、冲突、间距违反和碰撞事件记录。
5. 为 GA 增加逐代 trace。
6. 为 SA 增加逐迭代或抽样 trace。
7. 编写可中断后继续的批量运行器。
8. 先运行“1 个场景 × 2 个算法 × 2 个 seed”的 smoke test。
9. 检查字段完整性、单位、运行时长和任务流一致性。
10. 执行完整 135 次核心实验。
11. 自动生成 run-level CSV、聚合 CSV、置信区间和统计检验结果。
12. 使用聚合数据更新论文和答辩 PPT 图表。

## 12. 对应代码位置

主要修改位置包括：

- `scripts/run_experiments.py`：实验矩阵、seed、任务流、恢复运行和 manifest；
- `controllers/factory_supervisor/metrics_collector.py`：任务、机器人和安全指标；
- `controllers/factory_supervisor/schedulers.py`：GA/SA 优化轨迹；
- `controllers/factory_supervisor/factory_supervisor.py`：状态时间、安全事件和运行元数据；
- `controllers/factory_supervisor/task_generator.py`：固定任务流导入与导出；
- `presentation_data_analysis.py`：聚合统计、置信区间和图表生成。

## 13. 答辩前最低数据要求

至少应完成：

- 每个核心算法—场景组合 5 个完整 seed；
- 所有 JSON 明确记录 seed 和运行完整性；
- 同 seed 算法之间使用同一任务流；
- 计算均值、标准差和 95% CI；
- 保存 GA/SA 中间优化轨迹；
- 区分预测冲突、安全间距违反和物理碰撞；
- 报告 productive utilisation；
- 对最佳 RL 模型完成至少一组奖励消融实验。

完成这些数据后，才能对“RL 是否提高吞吐量、降低等待时间、改善利用率和扩展性”给出统计上更可靠的结论。
