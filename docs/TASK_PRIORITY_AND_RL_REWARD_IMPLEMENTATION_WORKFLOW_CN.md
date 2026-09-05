# 任务优先级、Deadline 与 RL Reward 修复实施 Workflow

> 状态：待实施  
> 适用范围：当前 Webots 多机器人单任务调度、抽象 RL 训练环境及 A/B/C 场景评估  
> 核心目标：在不削弱现有安全约束的前提下，引入可解释的任务优先级、完成时限、ETA、迟到损失和一致的 RL 奖励闭环  
> 非本轮目标：MES 正式接入、工位/充电站完整时间窗预约、多任务队列、任务合并拆分、RL 边评分矩阵、Assignment 与 MAPF 联合最优

## 1. 最终方案与边界

本轮采用以下分层结构：

```text
任务字段与 deadline 语义
    -> 统一 ETA / slack / tardiness
    -> 硬约束可行性与 action mask
    -> Deadline-aware 确定性调度基线
    -> RL 在相同合法 robot-task 动作中选择
    -> 批次候选校验与 best-found feasible subset 提交
    -> assignment 到 completion 的因果事件链
    -> 按时完成、迟到、等待增量、失败和终局奖励
```

强制边界：

- 安全、路径可达性、机器人状态、电量下限和任务状态合法性必须使用硬约束，不允许用 reward 权衡。
- `deadline` 统一表示“卸货服务完成时限”，不是到达交付坐标的时限。
- 内部优先级数值统一为“数值越大越紧急”。
- 第一版保留当前离散 `robot_slot * task_slot` 动作，不把现有 DQN、SARSA、PPO 强制改成边评分模型。
- `END_BATCH` 与 `WAIT` 必须语义分离；如第一版尚未启用批量 RL，则不得伪造 `END_BATCH` 行为。
- 固定步长 `gamma` 不再直接用于不等时间间隔的事件转移；必须记录 `elapsed_seconds` 并使用时间相关折扣。
- 训练、离线回放和 Webots 事件必须使用同一奖励纯函数和同一版本化契约。
- RL 失败、超时、非法输出或契约不兼容时，使用 Deadline-aware Hungarian 安全兜底。

## 2. 本轮最小任务模型

在 `TransportTask` 中新增或明确：

```python
priority_rank: int                 # 数值越大越紧急
target_completion_time: float | None
deadline: float | None             # 卸货服务完成时限
deadline_type: str                 # none / soft / hard
deadline_source: str               # simulation_sla / manifest / external
late_penalty_per_second: float
pickup_service_time: float
delivery_service_time: float
cargo_state: str                   # not_picked / onboard / delivered
reassignment_count: int
```

保留现有状态枚举的兼容性；本轮若不扩展完整任务状态机，至少保证：

- `PENDING/ASSIGNED` 时 `cargo_state=not_picked`。
- 取货完成后 `cargo_state=onboard`。
- 完成后 `cargo_state=delivered`。
- `cargo_state=onboard` 时，低电量或异常处理不得把任务直接当作普通未取货任务重新排队。

建议仿真优先级映射：

```python
CRITICAL = 400
URGENT = 300
NORMAL = 200
BACKGROUND = 100
```

任务 manifest 必须直接保存 deadline 输入，不得在训练器、调度器和 Webots 中分别随机生成。

## 3. ETA、Slack 与业务损失定义

第一版统一使用：

```text
expected_completion =
    current_time
  + empty_route_time
  + pickup_service_time
  + loaded_route_time
  + delivery_service_time
  + congestion_buffer

conservative_completion =
    expected_completion
  + max(5 s, travel_time * 0.15)

soft_slack = target_completion_time - expected_completion
hard_slack = deadline - conservative_completion
tardiness = max(0, actual_completion_time - deadline)
```

规则：

- 没有 deadline 时，相应 slack 和 tardiness 为 `None`，不得用 `0` 冒充。
- 所有时间单位为 Webots 仿真秒。
- ETA 输入只能使用决策时可见信息，不允许读取未来实际拥堵或完成时间。
- `assignment.estimated_cost` 不得再直接冒充 `estimated_distance`；必须拆分纯路径距离、预计时间和组合成本。
- 第一版记录 `eta_expected` 与 `eta_conservative`；概率分位 ETA 留到后续版本。

## 4. RL 奖励规范

### 4.1 唯一结算原则

| 分量 | 唯一结算时点 |
|---|---|
| 合法分配 | 默认 0，不因“合法”额外奖励 |
| 空驶塑形 | assignment commit 时，使用归一化纯空驶距离 |
| 等待成本 | 仿真时间推进时，按 pending wait 增量 |
| 年龄救援 | 分配长期等待任务时，小额且封顶 |
| 首次硬 deadline 违约 | 第一次跨过 deadline 时一次 |
| 迟到时间 | 任务完成时按实际 tardiness 一次 |
| 完成价值与按时奖励 | 任务完成时一次 |
| 改派 | 改派提交时一次 |
| 可重试/最终失败 | 进入对应失败状态时一次 |
| episode 未完成 | 终止时仅结算尚未结算的剩余损失 |

禁止对同一迟到同时执行“每步全量扣罚、完成时全量扣罚、终局再次全量扣罚”。

### 4.2 第一版建议尺度

```python
completion = 5.0
on_time = 3.0
hard_breach_once = -8.0
tardiness = -2.0

valid_assignment = 0.0
raw_priority_bonus = 0.0
empty_distance = -0.10
wait_increment = -0.02
age_rescue = 0.20

reassignment = -0.25
replan = -0.05
deadlock_recovery = -1.0
failed_retryable = -2.0
failed_final = -8.0
post_pickup_abort = -12.0

invalid_action = -5.0
avoidable_wait = -1.0
forced_wait = 0.0
collision = -100.0
```

连续值先归一化并封顶：

```python
distance_units = min(empty_distance_metres / 30.0, 3.0)
wait_units = min(wait_increment_seconds / 120.0, 2.0)
age_units = min(task_age_seconds / 120.0, 2.0)
tardiness_units = min(tardiness_seconds / 120.0, 3.0)
```

业务优先级不作为 assignment 即时奖金，而是通过完成价值和迟到损失体现。第一版业务权重可为：

```python
CRITICAL = 4.0
URGENT = 2.5
NORMAL = 1.0
BACKGROUND = 0.5
```

这些权重是待校准初值，不得以累计 reward 单独证明其合理性。

### 4.3 时间相关折扣

每条 transition 必须保存：

```python
elapsed_seconds
bootstrap_discount
```

统一定义：

```python
bootstrap_discount = gamma_base ** (elapsed_seconds / reference_seconds)
```

默认候选值：

```text
gamma_base = 0.99
reference_seconds = 10.0
```

所有 DQN、SARSA、SARSA(λ)、PPO/GAE 和 n-step return 必须明确是否支持该折扣。未支持的算法不得声称使用了统一的 deadline reward 契约。

## 5. 每一步的强制质量门禁

每个 Phase 必须严格按以下顺序执行：

1. 实现本 Phase，禁止夹带下一 Phase 的大范围重构。
2. 静态检查和目标专项测试。
3. **Code Review 1：正确性审查**。
4. 修复 Review 1 的全部阻断项，并记录处理结果。
5. 重新执行专项测试。
6. **Code Review 2：回归、实验可信性和故障注入审查**。
7. 修复 Review 2 的全部阻断项。
8. 执行本 Phase 完整测试门禁和全量回归。
9. 写出证据记录，状态只能为 `PASS` 或 `FAIL_CONTINUE`。
10. `PASS` 后才允许进入下一 Phase。

两轮 review 不能使用同一清单重复计数。

### Code Review 1 固定清单

- 需求和字段语义是否无歧义。
- 时间、距离、电量和 reward 单位是否一致。
- 状态转换、空值、边界值和异常处理是否正确。
- 是否保留既有接口兼容性或完成版本升级。
- 是否有重复奖励、未来信息泄漏或符号方向错误。
- 硬约束是否仍位于 mask/validator，而非 reward。
- 每个新增分支是否有单元测试。

### Code Review 2 固定清单

- Review 1 修复是否引入回归。
- 抽象训练、standalone、Webots 是否语义一致。
- 固定 seed 是否可重复。
- checkpoint、manifest、contract fingerprint 是否正确失效或兼容。
- fallback 成绩是否可能被错误归入 RL。
- 故障、取消、超时、低电量和 episode 终止是否正确结算。
- 性能预算、内存增长和事件账本上限是否可接受。
- 指标能否揭示 reward hacking，而不是只报告平均值。

## 6. 分步实施

### Phase 0：冻结基线与实验输入

实现：

- 固定当前 Git 状态、Python/Webots 版本和依赖版本。
- 为 A/B/C 生成固定任务 manifest，保存 arrival、priority 和当前任务位置。
- 至少冻结 FCFS、Hungarian 和一个当前最佳 RL checkpoint 的结果。
- 保存 throughput、完成率、等待时间、距离、安全事件、fallback、推理延迟和 reward 分量。

Code Review 1：

- 检查 manifest 是否真正驱动训练和 Webots，而不是仅作记录。
- 检查初始位置、电量、任务流和 seed 是否一致。
- 检查当前 priority 方向和所有使用点。

Code Review 2：

- 使用同一 seed 重跑两次，核对 manifest hash 和关键输入完全一致。
- 检查 baseline 中实际 policy commit 与 fallback 的归属。
- 检查旧指标是否足以与后续 deadline 指标并列比较。

测试与验收：

```powershell
python -m pytest -q
python scripts/generate_experiment_manifest.py --help
```

- 基线结果和 manifest hash 可重现。
- 未改变运行行为。

### Phase 1：任务字段、deadline 语义与 manifest

实现：

- 扩展 `TransportTask`、训练场景任务和 `ManifestTask`。
- deadline 在 manifest 中确定性生成或显式提供。
- 明确 deadline 是卸货完成时限。
- 增加 priority rank、deadline type/source、服务时间、late penalty 和 cargo state。
- 旧 manifest 必须明确拒绝或通过有版本号的迁移器加载，禁止静默补默认值。

Code Review 1：

- 检查 dataclass 默认值、序列化、排序、hash 和时间单位。
- 检查“数值越大越紧急”在调度、路径路权和指标中的一致性。
- 检查无 deadline、deadline 等于 arrival、已经过期等边界。

Code Review 2：

- 检查训练/Webots读取同一 manifest 后字段逐项相同。
- 检查旧 checkpoint 和旧 manifest 的兼容策略不会静默污染实验。
- 检查 cargo state 在取货、交付、低电量中止中的一致性。

测试文件建议：

- `tests/test_task_deadline_model.py`
- 扩展 `tests/test_experiment_manifest.py`
- 扩展 `tests/test_initial_battery_contract.py`

验收：字段 round-trip 零差异；相同 seed 生成相同任务 hash；无 deadline 任务不产生虚假迟到。

### Phase 2：统一 ETA、Slack 与指标

实现：

- 新增纯函数 ETA 计算模块，训练和运行共同调用。
- 拆分 empty distance、loaded distance、route time、service time、buffer 和 pair cost。
- 记录 expected/conservative ETA、soft/hard slack、实际 tardiness 和 ETA error。
- 指标增加各优先级按时率、总加权迟到、最大迟到、P95等待和未完成任务。

Code Review 1：

- 检查 ETA 不使用未来真实数据。
- 检查 `None`、不可达、零距离、服务时间和负 slack。
- 检查 `estimated_cost` 不再被错误标记为纯距离。

Code Review 2：

- 对相同 snapshot 比较训练与运行 ETA 输出。
- 注入路径不可达、拥堵、零服务时间和极端 deadline。
- 检查指标分母包含全部到达任务，不能只统计成功任务。

测试文件建议：

- `tests/test_task_eta.py`
- `tests/test_deadline_metrics.py`
- 扩展 `tests/test_rl_contract_parity.py`

验收：固定 snapshot 的 ETA/slack 零差异；实际完成后迟到只结算一次；指标无选择性分母。

### Phase 3：硬约束、可行性与 Deadline-aware Hungarian 基线

实现：

- 将机器人状态、电量、路径、任务状态和 deadline 字段统一传入 cost matrix。
- 硬约束只通过 feasibility/mask 表达。
- 实现 deadline-aware 确定性匹配，优先降低业务违约损失，再优化空驶和等待。
- 保留旧 Hungarian 作为可对照 baseline，或通过版本化配置切换。

Code Review 1：

- 检查所有调度器使用相同 feasibility contract。
- 检查不可避免迟到不会被误判为无可行任务。
- 检查高 priority 但宽 deadline 不会无条件压倒临期任务。

Code Review 2：

- 进行反例测试：一个不可挽救关键任务与多个可按时紧急任务。
- 检查低优任务老化不会破坏硬 deadline。
- 对照枚举小规模最优解，验证匹配目标方向正确。

测试文件建议：

- `tests/test_deadline_scheduler.py`
- `tests/test_deadline_feasibility.py`
- 扩展 `tests/test_evaluation_gate.py`

验收：小规模枚举用例与确定性调度输出一致；硬约束零绕过；相同输入输出确定。

### Phase 4：批次选择与可行子集提交

实现：

- 修复 Supervisor 仅执行 `decision.assignments[0]` 的行为。
- 调度器输出完整候选批次。
- 路径与资源校验后，按固定排序提交时间预算内得到的 `best-found feasible subset`。
- 每批记录 `batch_id`、snapshot version、候选数、提交数和拒绝原因。
- 部分失败不要求整批回滚，也不得伪称取得全局最大可行子集。

Code Review 1：

- 检查同一机器人和任务不会重复提交。
- 检查部分规划失败、命令发送失败和状态变化时的回滚。
- 检查批量提交不会绕过已有 route/reservation owner。

Code Review 2：

- 注入第二/第三条匹配失败，确认其他合法匹配仍可提交。
- 检查提交顺序不会依赖无序 dict/set。
- 检查规划超时时返回当前合法子集，并记录降级原因。

测试文件建议：

- `tests/test_batch_assignment_transaction.py`
- 扩展 `tests/test_joint_plan_transaction.py`
- 扩展 `tests/test_supervisor_prediction.py`

验收：多匹配可实际提交；失败边可回滚并进入 TTL；无重复任务/机器人；批次可审计。

### Phase 5：统一奖励事件与唯一结算

实现：

- 扩展 `RLEventLedger` 和 `reward_for_event()`。
- 取消原始 priority 即时奖金，将合法分配奖励降为 0。
- 等待改为时间推进增量惩罚；年龄救援单独封顶。
- 增加 on-time、first breach、tardiness、retryable/final failure、reassignment、post-pickup abort 和 terminal pending。
- 每个事件保存 `decision_id`、`batch_id`、任务版本和必要原始量。

Code Review 1：

- 建立奖励结算表，逐项检查事件是否重复或遗漏。
- 检查所有连续值归一化、封顶和符号。
- 检查 forced wait、productive wait 和 avoidable wait 的区别。

Code Review 2：

- 使用固定事件轨迹做在线/离线 reward replay 一致性测试。
- 注入重复事件、乱序事件和episode中止，确认幂等或明确拒绝。
- 检查拒绝、取消或拖到终局不能改善表面reward。

测试文件建议：

- 扩展 `tests/test_rl_event_ledger.py`
- `tests/test_deadline_reward.py`
- `tests/test_reward_accounting.py`

验收：固定事件轨迹在线/离线累计reward完全一致；每个业务损失只结算一次；reward分量可单独报告。

### Phase 6：时间相关折扣与异步信用链

实现：

- transition 增加 `elapsed_seconds` 与实际 `bootstrap_discount`。
- 更新 n-step、DQN target、SARSA target、SARSA(λ) eligibility 和 PPO/GAE 时间折扣。
- assignment 到 pickup/completion/failure 使用稳定 `decision_id` 建立因果链。
- 多机器人异步完成不得把奖励简单归给最近动作。

Code Review 1：

- 手工推导 0、1、10、100 秒间隔的折扣结果并对照代码。
- 检查 terminal transition 不 bootstrap。
- 检查 n-step 累积使用每段折扣乘积，而非固定 `gamma ** n`。

Code Review 2：

- 构造三个机器人乱序完成轨迹，检查每个奖励归属。
- 检查 PPO 旧策略概率、value 和 policy version 在延迟完成后仍可追溯。
- 检查不同算法若未支持新契约会被 registry/gate 拒绝。

测试文件建议：

- `tests/test_time_aware_discount.py`
- `tests/test_async_reward_credit.py`
- 扩展 `tests/test_advanced_rl_common.py`
- 扩展 `tests/test_rl_model_registry.py`

验收：解析算例完全匹配；异步完成归因正确；旧模型不能被误加载为新契约模型。

### Phase 7：RL observation、动作语义与环境一致性

实现：

- 任务特征加入 priority、age、soft/hard slack、预计迟到和deadline类型。
- 机器人特征加入预计电量余量、任务/货物状态和状态新鲜度中本轮确有数据的部分。
- 全局特征加入 at-risk、breached 和待处理工作量。
- 明确 `NO_OP/WAIT` 规则；如引入批次RL，新增独立 `END_BATCH` 并升级action codec。
- 抽象环境加入服务时间、ETA buffer 和受控随机扰动。

Code Review 1：

- 检查特征顺序、归一化、mask和slot顺序。
- 检查无deadline任务的sentinel不会与“零slack”混淆。
- 检查 observation 不含未来数据。

Code Review 2：

- 对至少100个固定snapshot做训练/部署 observation和mask逐元素比较。
- 检查WAIT不会成为无限拖延漏洞。
- 检查contract fingerprint、environment version和checkpoint metadata全部升级。

测试文件建议：

- 扩展 `tests/test_rl_contract_parity.py`
- 扩展 `tests/test_manifest_training_parity.py`
- 扩展 `tests/test_rl_environment.py`（若当前不存在则新增）

验收：snapshot parity零差异；旧checkpoint默认拒绝；WAIT/NO_OP边界全部覆盖。

### Phase 8：训练流程、模型门控与 reward hacking 测试

实现：

- 重新训练正式支持的RL算法，不复用旧模型参数作为新结果。
- 训练、验证、最终测试和压力测试seed严格隔离。
- checkpoint选择使用validation seeds，不使用最终测试集。
- 增加奖励分量分布、fallback、invalid、WAIT、未完成任务和各优先级KPI。

Code Review 1：

- 检查训练脚本确实加载新manifest、reward和contract。
- 检查探索动作也经过mask。
- 检查恢复训练保留优化器、随机状态、环境版本和reward版本。

Code Review 2：

- 检查是否出现故意养老任务、偏爱短任务、拖到episode结束、频繁改派或依赖fallback。
- 检查模型选择不存在seed泄漏或测试集调参。
- 对关键reward权重执行至少±20%敏感性检查。

测试与验收：

- 训练 smoke test 能保存、恢复并确定性复现短轨迹。
- reward任一分量不得长期占总绝对reward的80%以上而无解释。
- fallback和invalid超过门限的checkpoint不得进入Webots排名。

### Phase 9：三层评估与Webots验收

按顺序执行，前一级失败不得启动后一级：

1. 抽象环境固定manifest评估。
2. standalone/shadow一致性评估。
3. Webots A/B/C 多seed评估。

每一级都比较：

- 旧基线。
- Deadline-aware Hungarian。
- 新RL原生决策。
- RL含fallback的实际运行结果，但两者必须分开报告。

Code Review 1：

- 检查所有算法使用相同任务流、初始电量、重调度机会和时间预算。
- 检查到达任务、拒绝任务、完成任务、未完成任务分母一致。
- 检查RL结果没有混入fallback贡献。

Code Review 2：

- 逐个审查失败seed，而非只看平均值。
- 检查置信区间、P95、最大值和优先级分层指标。
- 检查仿真提前终止、controller异常和缺失字段自动判FAIL。

最终硬门禁：

```text
collision_count == 0
pair_distance_violations == 0
nonphysical_recoveries == 0
状态/货物一致性错误 == 0
controller异常退出 == 0
manifest/contract/checkpoint provenance 完整
```

业务门禁：

- 新RL的硬deadline加权损失不得劣于Deadline-aware Hungarian的预先声明容差。
- 吞吐量不得出现未解释的显著下降。
- 最大等待和BACKGROUND饥饿不得恶化到预设上限之外。
- 推理与调度P95延迟满足实时预算。
- 至少报告3个seed；正式结论建议5个以上seed。

### Phase 10：发布、默认关闭与回滚

实现：

- 新行为通过版本化配置或特性开关接入，初始默认关闭。
- 结果保存代码版本、manifest hash、reward version、contract fingerprint、checkpoint hash和Webots版本。
- 提供一键切回旧调度语义和Deadline-aware Hungarian的方法。
- 旧结果不得在新契约下继续参与排名。

Code Review 1：

- 检查默认配置不会意外加载实验checkpoint。
- 检查回滚不破坏任务或货物状态。
- 检查配置组合无矛盾状态。

Code Review 2：

- 从干净进程执行开启、关闭、checkpoint错配和运行中fallback演练。
- 检查报告能追踪每个完成任务的真实决策算法。
- 检查文档、CLI和实际默认值一致。

验收：旧行为可恢复；新结果可重现；任何缺少provenance的结果自动失效。

## 7. 每个 Phase 的测试命令门禁

根据改动范围执行专项测试后，每个 Phase 至少执行：

```powershell
git diff --check
python -m compileall -q controllers scripts tests
python -m pytest -q
```

涉及RL契约、奖励或训练时额外执行：

```powershell
python -m pytest -q tests/test_rl_contract_parity.py
python -m pytest -q tests/test_rl_event_ledger.py
python -m pytest -q tests/test_rl_model_registry.py
python -m pytest -q tests/test_manifest_training_parity.py
python -m pytest -q tests/test_advanced_rl_integration.py
```

涉及Supervisor批量提交、路径或安全时额外执行：

```powershell
python -m pytest -q tests/test_supervisor_prediction.py
python -m pytest -q tests/test_joint_plan_transaction.py
python -m pytest -q tests/test_motion_coordination.py
python -m pytest -q tests/test_priority_yield_resume.py
python -m pytest -q tests/test_metrics_safety.py
```

若文档列出的新测试文件尚未在对应Phase创建，该Phase不得标记PASS。

## 8. 阶段验收记录模板

```text
Phase：
代码/文档改动范围：
Git commit或diff标识：
使用的manifest及SHA-256：
使用的checkpoint及SHA-256：

静态检查：
专项测试及结果：

Code Review 1：
- 审查清单：正确性/接口/边界/奖励唯一结算/硬约束
- 发现问题：
- 修复结果：

Review 1修复后测试：

Code Review 2：
- 审查清单：回归/一致性/故障注入/性能/实验污染
- 发现问题：
- 修复结果：

全量回归：
Webots/训练证据：
遗留风险：
验收状态：PASS / FAIL_CONTINUE
```

## 9. Reward Hacking 专项清单

每次新奖励训练后必须主动检查：

- 是否故意延后任务以获取年龄奖励。
- 是否只做短任务刷完成数量。
- 是否忽略长距离但关键的任务。
- 是否在episode结束前遗留困难任务。
- 是否通过拒绝、取消或fallback改善表面按时率。
- 是否频繁改派以刷新预测指标。
- 是否不必要地WAIT来保留未来运力。
- 是否牺牲少数任务以改善平均值。
- 是否依赖抽象环境中不存在的理想速度或零服务时间。
- 是否以更多replan、deadlock或交通等待换取表面deadline收益。

发现任一行为时，先检查事件语义、状态观测和动作约束；不得第一反应只调大惩罚。

## 10. 最终验收输出

最终报告必须同时给出：

```text
全部到达任务数
接受/拒绝/取消/完成/未完成任务数
总体及各优先级按时率
首次硬deadline违约数
总加权迟到、最大迟到、P95迟到
平均/P95/最大等待时间
吞吐量与完成率
空驶距离、总距离和能耗
reassignment/replan/deadlock/fallback/invalid/WAIT次数
碰撞、间距违规、非物理恢复和状态一致性错误
调度与推理P50/P95/最大耗时
ETA误差P50/P90/P95
每个算法的原生commit与fallback commit
manifest、checkpoint、contract、reward和代码版本信息
```

只有当硬安全门禁全部通过、两轮review证据完整、全量测试通过、结果可重现且RL原生成绩没有被fallback污染时，才能将本workflow标记为完成。

