# SA、GA、SARSA、PPO 与 DQN 优化实施方案

## 1. 目标与适用范围

本方案面向当前 Webots 智能工厂多机器人项目，目标不是单纯提高训练 reward，而是在相同任务流和运动协调层下，提高：

- 任务吞吐率和完成率；
- 平均等待时间与任务完成时间；
- 机器人工作负载均衡；
- 调度实时性与结果稳定性；
- 路径可执行性、最小机器人间距和死锁恢复能力；
- A/B/C 三种负载下的泛化能力。

优化对象分为两组：

1. 元启发式调度：Simulated Annealing（SA）、Genetic Algorithm（GA）。
2. 强化学习调度：SARSA、PPO、Double DQN。

## 2. 当前实现审查结论

### 2.1 共性问题

1. SA、GA 优化的是一个完整的一对一匹配排列，但 Supervisor 每轮只提交 `decision.assignments[0]`。因此“全局匹配最优”和“当前实际提交的第一个动作”可能不一致。
2. SA、GA 的适应度主要复用 `_pair_cost`，尚未充分刻画电池风险、预计路径拥堵、任务老化、工作量均衡和未来充电占用。
3. DQN/SARSA 使用 `rl-scheduling-v1` 抽象环境，而 PPO 使用另一套 126 维状态和独立训练模拟器，三者的状态、动作、奖励和 episode 定义不一致，不能直接公平比较。
4. 训练环境把任务分配近似为立即完成，和 Webots 中“取货—运输—交付—充电—避障”的时延存在 sim-to-sim gap。
5. 当前最终比较缺少完整的 A/B/C × 多随机种子实验，单次结果不能证明算法显著优越。

### 2.2 必须优先修复的问题

在 `rl_environment.py` 中当前奖励为：

```text
reward = completion + valid_assignment + priority
         - distance_cost + 0.02 × waiting_time
```

等待项是正数，策略会因为选择等待更久的任务获得额外奖励。如果本意是减少平均等待，应改成等待惩罚；如果本意是防止任务饥饿，则应该使用有上限的“任务老化优先级奖励”，不能直接奖励无限增长的等待时间。

推荐拆分为：

```text
reward = +10.0 × completed
         +0.5  × valid_assignment
         +0.5  × bounded_priority
         -0.08 × normalized_empty_distance
         -0.03 × normalized_waiting_increment
         -0.20 × congestion_risk
         -0.30 × battery_risk
         -100  × collision
```

任务防饥饿单独加入：

```text
age_bonus = min(waiting_time / 120, 2.0)
```

## 3. 统一优化目标

### 3.1 调度代价函数

所有算法使用同一个可解释的归一化代价：

```text
J = 0.30 × pickup_travel
  + 0.20 × delivery_travel
  + 0.15 × task_wait
  + 0.10 × congestion_risk
  + 0.10 × workload_imbalance
  + 0.10 × battery_risk
  + 0.05 × scheduling_latency
  + hard_safety_penalty
```

约束：

- 非空闲机器人不可接新任务；
- 电量低于 25% 不得分配普通任务；
- 无可行路径的机器人—任务对必须 mask；
- 碰撞、非法动作和命令发送失败使用硬惩罚；
- 安全指标不能通过提高吞吐率来抵消。

### 3.2 多目标选择规则

采用“安全优先的词典序选择”：

1. `pair_distance_violations == 0`；
2. `collision_count == 0`；
3. 最大化完成率；
4. 最大化吞吐率；
5. 最小化 P95 等待时间；
6. 最小化 workload CV；
7. 满足 P95 调度时间预算。

不建议一开始把所有指标压缩成一个分数，否则可能出现“用碰撞换吞吐”的错误最优解。

## 4. SA 优化方案

### 4.1 当前局限

- 初始解固定为任务原始顺序，质量依赖任务到达顺序；
- 邻域只有两点交换，搜索能力较弱；
- 固定初温 `10.0` 没有根据实际 cost 差值校准；
- 固定几何降温 `0.95`，5 ms 预算下可能过早冻结；
- 每个邻域完整重算 cost，浪费有限实时预算；
- 没有停滞重加热和多起点机制。

### 4.2 改进内容

1. 初始解使用 Hungarian 或 Greedy 结果，而不是 identity permutation。
2. 混合邻域：
   - 交换两个任务；
   - 插入移动；
   - 2-opt 片段反转；
   - 重新分配一个高等待任务。
3. 自适应初温：抽样 50–100 个邻域差值，令初始劣解接受率约为 0.8：

   ```text
   T0 = -mean(positive_delta) / ln(0.8)
   ```

4. 使用分段或自适应降温：接受率低于 0.1 时减慢降温；连续 100 次无改进时执行 `T ← 0.3 × T0` 重加热。
5. 使用增量 cost，只重算被交换任务和机器人的代价。
6. 保留 best-so-far，时间到立即返回当前历史最优解。
7. 对高负载场景 C 使用 2–4 个短多起点，而不是一个长链。

### 4.3 SA 搜索空间

| 参数 | 候选值 |
|---|---|
| 初始接受率 | 0.70, 0.80, 0.90 |
| cooling rate | 0.97, 0.985, 0.995 |
| minimum temperature | 0.001, 0.01, 0.05 |
| time budget | 2, 5, 10, 20 ms |
| no-improvement reheating | 50, 100, 200 次 |
| multi-start | 1, 2, 4 |

建议首选配置：自适应 `T0`、`cooling=0.985`、5 ms、100 次停滞重加热、2 个起点。

## 5. GA 优化方案

### 5.1 当前局限

- 当前实现没有真正的 crossover；后代主要来自 elite 的一次交换变异；
- 机器人顺序固定，只优化任务排列；
- 初始种群只有 identity 和随机排列，缺少高质量种子；
- 没有 tournament selection、多样性控制和自适应变异率；
- 完整匹配优化后只执行第一条 assignment，可能损失全局解价值。

### 5.2 染色体重构

优先方案是直接编码匹配对：

```text
chromosome = [(robot_slot, task_slot), ...]
```

通过 repair operator 保证：

- robot 不重复；
- task 不重复；
- 所有 pair 均满足可行性 mask；
- 不足的基因允许为空，避免强制分配低质量任务。

如果暂时保持任务排列编码，至少应在染色体内同时编码机器人排列。

### 5.3 改进内容

1. 种群注入：Hungarian、Greedy、Nearest Neighbour、等待时间优先各一个种子，其余随机生成。
2. Selection：3 或 4 个体 tournament selection。
3. Crossover：排列采用 OX 或 PMX；匹配对编码采用 uniform crossover + repair。
4. Mutation：swap、insert、scramble 三种随机选择。
5. 自适应变异：
   - 多样性高：`p_mutation = 0.10–0.15`；
   - 连续 5 代无提升：提高到 `0.30–0.40`；
   - 出现提升后恢复基础变异率。
6. Elitism 控制在 5%–10%，当前 25% 容易早熟。
7. 缓存 pair cost 矩阵，fitness 只做索引求和。
8. 对 Supervisor 采用以下二选一：
   - 原子提交整组无冲突 assignment；或
   - 适应度明确只优化“本轮会提交的首个动作 + 对后续队列价值的估计”。

### 5.4 GA 搜索空间

| 参数 | 候选值 |
|---|---|
| population | 24, 48, 96 |
| generations | 20, 50, 100 |
| crossover rate | 0.70, 0.85, 0.95 |
| mutation rate | 0.10, 0.20, 0.35 |
| elite ratio | 0.05, 0.10, 0.15 |
| tournament size | 2, 3, 4 |
| time budget | 5, 10, 20 ms |

建议首选配置：population 48、OX crossover 0.85、adaptive mutation 0.15→0.35、elite 10%、tournament 3、10 ms。

## 6. SARSA 优化方案

### 6.1 当前局限

当前 SARSA 状态只离散化全局前缀：pending ratio、idle ratio、congestion、feasible density、time。它没有直接看到：

- 哪台机器人更靠近哪个任务；
- 每台机器人的电量；
- 任务等待时间和优先级；
- robot–task pair cost；
- 动作对应关系的空间结构。

因此不同工厂状态会大量碰撞到同一个表格状态，但动作仍是 161 个固定 robot–task slot，Q 值难以泛化。

### 6.2 改进路径

短期保留 tabular SARSA：

1. 状态改为“候选 pair 级别”，只对 Top-K 可行 pair 排序后编码。
2. 增加最低 pair cost、平均电量、最大任务年龄和 workload CV 桶。
3. 使用 eligibility traces，升级为 SARSA(λ)，建议 `λ=0.8`。
4. 学习率按 state-action 访问次数衰减：

   ```text
   alpha(s,a) = alpha0 / sqrt(1 + N(s,a))
   ```

5. epsilon 使用分段退火，并保留 `epsilon_min=0.02`。
6. 对稀有高负载状态增加 prioritized episode sampling。

中期建议改为 Linear SARSA 或 tile coding，避免 Q-table 状态碰撞，同时保持可解释性。

### 6.3 推荐搜索空间

| 参数 | 候选值 |
|---|---|
| alpha0 | 0.03, 0.05, 0.10 |
| gamma | 0.95, 0.98, 0.99 |
| lambda | 0.0, 0.6, 0.8, 0.9 |
| epsilon decay | 0.997, 0.999, 分段退火 |
| final epsilon | 0.01, 0.02, 0.05 |

建议先训练至少 10,000 episode，并用独立 validation seeds 选择 checkpoint，而不是使用训练末尾模型。

## 7. DQN 优化方案

### 7.1 当前优势与局限

当前实现已经包含 Double DQN、target network、Huber loss、Adam、梯度裁剪和 action mask，这是可靠基础。但仍存在：

- 网络宽度只有 64，对约 270 维状态和 161 动作可能不足；
- replay 为均匀采样，碰撞/高等待/稀有拥堵样本利用率低；
- target 每 250 步硬更新可能引起波动；
- 仅一层标量 Q 输出，未使用 dueling 结构；
- 单步 TD 对延迟完成奖励传播较慢；
- 抽象环境立即完成任务，削弱了长期信用分配。

### 7.2 改进内容

1. 网络升级为 Dueling Double DQN：共享编码后分别输出 `V(s)` 和 `A(s,a)`。
2. hidden size 比较 128、256；建议先用两层 256。
3. 使用 Prioritized Experience Replay：`alpha=0.6`，`beta=0.4→1.0`。
4. 使用 n-step return，推荐 n=3 或 n=5。
5. target network 改为 soft update：`tau=0.005`，或比较硬更新 500/1000 步。
6. 对 masked illegal action，在 online 和 target 选择阶段均设为负无穷；继续保留当前安全 mask。
7. reward clip 采用 `[-10, 10]` 或先标准化，碰撞硬惩罚单独保留终止信号。
8. replay warm-up 提高到 2,000–5,000 transition。
9. epsilon 在总训练步的 30%–50% 内由 1.0 退火到 0.05，之后保持 0.02–0.05。
10. 每 20 个训练 episode 在固定 validation seed 集上评估，保存最佳完成率约束下的最高吞吐模型。

### 7.3 推荐首轮配置

```text
hidden_size       = 256
learning_rate     = 1e-4
batch_size        = 128
replay_capacity   = 100000
warmup_steps      = 5000
n_step            = 3
PER alpha/beta    = 0.6 / 0.4→1.0
gamma             = 0.99
soft target tau   = 0.005
gradient clip     = 5.0
```

## 8. PPO 优化方案

### 8.1 当前局限

1. PPO 只选择“哪台空闲机器人接最高优先级任务”，动作空间与 DQN/SARSA 的 robot–task pair 不一致。
2. 当前训练模拟器与真实 Supervisor 状态机不一致，未完整模拟充电、冲突等待、应急制动和实际任务队列演化。
3. update 只在 buffer 满时发生；episode 结束但 buffer 未满的轨迹可能不能及时利用。
4. 当前 NumPy 实现逐样本更新，缺少标准化 value target、KL early stop、learning-rate schedule 等稳定措施。
5. checkpoint 选择主要依赖最终模型，缺少与 DQN/SARSA 一致的 held-out validation 机制。

### 8.2 改进内容

1. 统一动作空间为 masked robot–task pair + NO_OP，和 DQN/SARSA 相同。
2. 状态改用统一 `SchedulingEnvironment.observe()`，或采用集合编码：
   - robot encoder；
   - task encoder；
   - pair attention / pointer policy。
3. 每个 rollout 结束均更新，不等待跨 episode 填满 buffer。
4. advantage 标准化，并对 value loss 使用 clipped value objective。
5. 增加 approximate KL 监控；`KL > 0.015–0.03` 时提前停止当前 epoch。
6. entropy coefficient 从 0.02 线性衰减到 0.001。
7. learning rate 从 `3e-4` 线性衰减到 `3e-5`。
8. clip epsilon 比较 0.1、0.2、0.3，默认 0.2。
9. 使用 8–16 个并行抽象环境收集轨迹，提高样本多样性。
10. Curriculum 不只增加机器人数量，还要逐渐加入：
    - 更高任务频率；
    - 随机初始电量；
    - 路径阻塞和重规划成本；
    - 充电站竞争；
    - 位置和任务分布扰动。

### 8.3 推荐首轮配置

```text
rollout steps      = 2048
parallel envs      = 8
batch size         = 256
epochs             = 5–10
learning rate      = 3e-4 → 3e-5
gamma              = 0.99
GAE lambda         = 0.95
clip epsilon       = 0.2
target KL          = 0.02
entropy coefficient= 0.02 → 0.001
max grad norm      = 0.5
```

## 9. 统一训练与实验协议

### 9.1 数据划分

禁止用 test seed 选模型：

- Train seeds：`0–9999`；
- Validation seeds：`200000–200099`；
- Test seeds：`300000–300099`；
- OOD seeds：`400000–400099`，同时改变任务到达率和初始电量分布。

### 9.2 两阶段评估

阶段一：快速抽象环境筛选。

- 每组超参数至少 5 个训练 seed；
- validation 至少 30 个 episode；
- 使用 Successive Halving 或 Optuna TPE 淘汰差配置；
- 只保留 3–5 个候选进入 Webots。

阶段二：Webots 最终验证。

- 算法：FCFS、Hungarian、SA、GA、SARSA、DQN、PPO；
- 场景：A、B、C；
- 每个组合至少 10 个共同随机种子；
- 每次 1800 s；
- 所有算法使用完全相同的任务流、初始电量和机器人初始位置；
- 共需 `7 × 3 × 10 = 210` 次正式运行。

### 9.3 报告统计

- 给出 mean、standard deviation、95% bootstrap CI；
- 算法两两比较使用 Wilcoxon signed-rank test；
- 多算法比较进行 Holm correction；
- 同时报告效果量，不只报告 p-value；
- RL 报告 inference latency，不把训练时间混入在线调度延迟。

## 10. 消融实验

| 编号 | 消融内容 | 目的 |
|---|---|---|
| A1 | 去掉 battery risk | 验证电池特征是否减少任务中断 |
| A2 | 去掉 congestion term | 验证调度是否真正改善运动协调 |
| A3 | 去掉 workload term | 验证均衡奖励的贡献 |
| A4 | DQN 去掉 PER | 测量优先经验回放收益 |
| A5 | DQN 去掉 dueling | 测量网络结构收益 |
| A6 | PPO 去掉 curriculum | 验证课程学习收益 |
| A7 | SARSA λ=0 vs λ=0.8 | 验证 eligibility trace |
| A8 | GA 无 crossover vs OX/PMX | 证明交叉算子的作用 |
| A9 | SA 固定温度 vs 自适应温度 | 验证温度校准与重加热 |
| A10 | RL 无安全 mask | 只在抽象环境中测非法动作率，不允许进入 Webots |

## 11. 实施顺序与工期

### Phase 0：评价可信性（2–3 天，P0）

- 修正 waiting reward 符号/语义；
- 统一指标定义和结果 JSON schema；
- 固化 train/validation/test/OOD seeds；
- 为相同任务流建立 replayable task trace；
- 增加 collision count、emergency braking time、charge waiting 等指标。

验收：同一算法同一 seed 可重复；test seed 不参与 checkpoint 选择。

### Phase 1：SA/GA（3–5 天，P0）

- pair-cost 缓存；
- SA 自适应温度、混合邻域、多起点；
- GA 重构染色体、加入 crossover/repair/tournament；
- 解决“完整匹配优化但只提交首项”的契约问题。

验收：P95 调度延迟满足 20 ms；在 B/C 中至少一项效率指标显著优于原版本，安全不退化。

### Phase 2：统一 RL 环境（4–7 天，P0）

- PPO/DQN/SARSA 统一 observation、action mask、reward 和 episode；
- 抽象环境加入任务持续时间、电池和拥堵近似；
- checkpoint metadata 升级到 `rl-scheduling-v2`。

验收：三种算法可在相同输入上输出同一动作语义，旧 checkpoint 被明确拒绝而不是静默加载。

### Phase 3：SARSA/DQN/PPO 优化（1–2 周，P1）

- SARSA(λ) / tile coding；
- Dueling Double DQN + PER + n-step；
- PPO masked pair policy + KL early stop + parallel curriculum；
- 自动调参与 validation checkpoint selection。

验收：validation 五随机种子稳定，无非法动作；训练曲线无 NaN，最佳模型可复现。

### Phase 4：Webots 正式实验（1–2 周，P1）

- 执行 210 次统一实验；
- 输出 CSV/JSON、置信区间、显著性与效果量；
- 生成论文表格和 PPT 图表。

验收：完成 A/B/C 全矩阵，不以单次最好结果作为最终结论。

## 12. 最终成功标准

优化后的 RL 算法必须至少满足：

1. 所有正式测试中碰撞数为 0；
2. 非法调度输出为 0，fallback rate 小于 1%；
3. Scenario B/C 相对 Hungarian：
   - 吞吐率提高目标 ≥ 8%；或
   - P95 等待时间降低目标 ≥ 10%；
4. workload CV 不比 Hungarian 恶化超过 5%；
5. 在线 P95 inference latency：
   - SARSA < 1 ms；
   - DQN/PPO < 5 ms；
6. 至少 10 个测试 seed 的 95% CI 支持结论；
7. OOD 场景性能下降不超过同类基线的 10 个百分点。

SA/GA 的成功标准：在 20 ms 在线预算内，比原实现获得更低的匹配 cost，并在 Webots B/C 中改善吞吐或等待时间，同时不增加安全距离违规。

## 13. 推荐优先级结论

最先做的不是扩大网络或增加训练 episode，而是：

1. 修正奖励函数和统一实验协议；
2. 修复 GA 缺少 crossover 与批量匹配提交契约；
3. 统一 PPO、DQN、SARSA 的环境和动作空间；
4. 再进行算法级超参数优化；
5. 最后用多随机种子 Webots 实验证明优势。

预计最具研究价值的最终组合是：

- **确定性安全基线：Hungarian**；
- **实时元启发式：改进 SA**；
- **高质量组合搜索：改进 GA**；
- **可解释学习基线：SARSA(λ)**；
- **离散调度主模型：Dueling Double DQN + PER**；
- **长期研究主模型：Masked PPO / Attention PPO**。
