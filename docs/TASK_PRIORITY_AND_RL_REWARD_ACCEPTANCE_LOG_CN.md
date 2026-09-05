# 任务优先级与强化学习奖励改造验收记录

日期：2026-08-30  
对应流程：`docs/TASK_PRIORITY_AND_RL_REWARD_IMPLEMENTATION_WORKFLOW_CN.md`

## 1. 验收结论

本轮已完成任务优先级、完成时限、ETA、调度成本、RL 状态/动作/奖励、按实际时间折扣、事件账本、指标、训练入口、运行时安全约束和实验入口的端到端改造。

实现验收通过；生产效果认证仍须使用正式训练预算、多随机种子和真实业务 SLA 数据完成离线/影子/灰度门禁。本记录不把短训练 smoke 模型声明为生产模型。

## 2. 最终业务与算法契约

- 优先级采用离散 `priority_rank` 表示业务重要性，不再直接把优先级数值作为即时奖励。
- 时限采用绝对 `deadline`，并保留 `deadline_type`、`deadline_source`、目标完成时间和逾期惩罚权重。
- 无 deadline 的任务不产生虚假迟到；到期未完成任务单独计入 overdue unfinished。
- 调度以机器人到取货点、取货到交付点、速度、拥堵、服务时长和不确定性形成 ETA。
- 调度成本同时考虑空驶、预计逾期、紧迫度、等待老化和业务权重；安全可行性始终是硬约束。
- RL 奖励按可审计事件结算：按期完成、逾期完成、首次违约、增量等待、空驶、失败、重分配、重规划、死锁恢复和终局未完成。
- 等待惩罚按增量结算，违约只在首次跨线结算，避免重复累计和策略利用。
- 折扣按实际经过时间计算；DQN、SARSA、SARSA(λ)、PPO、A2C、离散 SAC、QR-DQN、Rainbow 的相关路径已接入。
- 取货后的货物不能因低电量被当作普通待分配任务重新入队。
- 无候选/无可行配对属于正常 no-op，不计为 RL 失败或 fallback。
- 旧环境契约 checkpoint 会因版本/维度/指纹不匹配被拒绝，避免静默加载错误模型。

## 3. 分阶段双轮代码评审

### Phase 0-2：基线、任务模型、ETA

第一轮：检查兼容性、字段默认值、manifest 可复现性和 deadline 空值语义。发现无 deadline 可能被错误转换为迟到，已修复为显式 `None` 传播。

第二轮：检查 ETA 与 Webots 实际行为一致性、优先级采样一致性和风险判定。修复服务时长默认值与仿真瞬时服务不一致、priority/rank 两次随机采样不一致、at-risk 应采用最佳可行 slack 而非最差 slack。

### Phase 3-4：调度策略、状态动作与安全层

第一轮：检查批量匹配是否只提交首项、候选失效和部分提交语义。改为保存批次、逐项重新校验、允许安全的部分提交。

第二轮：检查 no-op、fallback、低电量和载货状态。修复正常无候选被累计为失败、载货任务被重新入队，以及调度重试过密问题。

### Phase 5-6：奖励、账本与时间折扣

第一轮：逐事件核对唯一性、归属和正负号。移除原始优先级即时奖励，将有效分配奖励置零，完成奖励改为结果导向。

第二轮：检查奖励投机、长等待净收益、终局逃逸和不等时间步折扣。提高增量等待惩罚，使长期等待不能被老化奖励抵消；加入首次违约、终局未完成和 elapsed-time discount；修复 no-op 事件缩进导致的错误记账。

### Phase 7：指标与可观测性

第一轮：检查分母和删失数据。按全部到达 deadline 任务计算按期率，并分别报告已完成按期、违约和到期未完成。

第二轮：检查账本规模、decision/batch 关联和重复事件。加入序号与关联字段，将运行时等待事件按约一秒聚合并设置账本上限，避免长仿真事件膨胀。

### Phase 8：训练与回放兼容性

第一轮：检查 replay、N-step、GAE 和各算法 bootstrap 是否真正使用实际时间折扣。已完成逐转移 discount 传播。

第二轮：检查旧模型误加载、CLI 旧参数和训练/运行契约一致性。环境升级为 `rl-scheduling-v7-deadline-time-aware`，旧 checkpoint 拒绝加载，旧 priority reward 参数标记为弃用。

### Phase 9-10：集成、回归与发布边界

第一轮：执行编译、diff whitespace、全量测试、训练 smoke、模型审计和抽象多种子验证；修复评审中发现的兼容性问题。

第二轮：复核 Webots 结果、安全指标和统计解释。Hungarian 120 秒场景完成且无调度安全/约束异常；历史 DQN 1800 秒运行完成 51/53 且 51 个均按期，但该结果产生于 fallback 统计修复前，只用于发现统计缺陷，不作为新版模型效果认证。续验时定位到 DQN 超时并非推理性能问题，而是相对 checkpoint 路径在 Webots 控制器工作目录下失效；同时发现控制器输出未转发以及 GBK 控制台无法打印替换字符。三项均已修复并完成原生 Webots 重跑。

续验第一轮 review：对 checkpoint 加载和推理做独立基准，排除模型计算瓶颈；加入 Webots `--stdout --stderr` 和 Python 无缓冲输出后，捕获到控制器因相对路径找不到 checkpoint 而退出。修复为启动前展开、绝对化并验证 checkpoint 路径。

续验第二轮 review：修复 Webots 输出中的 Unicode 替换字符触发 Windows GBK `UnicodeEncodeError`，增加 console-safe 输出和回归测试；复跑 15 秒及 120 秒原生 DQN 场景，确认正常启动、退出和生成结果，不再依赖外层超时。

## 4. 测试证据

- `git diff --check`：通过，仅有 Git 的 LF/CRLF 转换提示，无 whitespace error。
- `python -m compileall -q controllers scripts tests`：通过。
- `python -m pytest -q`：续验后 `190 passed, 4 subtests passed`。
- SARSA、DQN、SARSA_LAMBDA 两回合 smoke：训练及模型审计通过。
- DQN 50 回合验证训练：完成并生成 v7 checkpoint。
- 抽象环境 3 seeds：SARSA/DQN 平均完成任务数 20，invalid action 为 0。
- Webots Scenario A / Hungarian / 120 秒：完成 1 个任务、按期，调度安全事件 0、pair constraint violation 0、nonphysical completion 0。
- Webots Scenario A / DQN / 15 秒：3 次 policy decision、3 次 DQN 原生提交，fallback、invalid output、安全事件和 pair violation 均为 0。
- Webots Scenario A / DQN / 120 秒：7 个任务到达、3 个原生提交、3 个到达取货点，fallback、invalid output、安全事件和 pair violation 均为 0；该 50 回合验证 checkpoint 在 120 秒窗口内交付完成数为 0，因此执行链路验收通过、策略效果门禁不通过，不可发布。

新增核心测试覆盖：

- deadline 模型、空 deadline 和迟到计算；
- ETA、拥堵和不确定性；
- deadline-aware 调度；
- 奖励重复结算、等待增量与终局结算；
- 实际时间折扣及 replay/N-step；
- RL 契约维度与旧 checkpoint 拒绝；
- benign no-op 不触发 fallback；
- 载货任务不可重新入队。

## 5. 发布门禁与后续运行要求

上线前必须满足以下条件：

1. 用正式 SLA 分布校准 deadline、业务权重和迟到成本，禁止直接把当前示例系数当成财务价值。
2. 使用固定 manifest、至少 5 个随机种子和相同仿真预算，对比 Hungarian、规则基线和候选 RL 模型。
3. 同时通过安全门禁、deadline 门禁、吞吐门禁、fallback 门禁和推理延迟门禁；单一总 reward 不能作为发布依据。
4. 先影子运行，再小流量灰度；保留规则调度器回退、模型版本锁定和一键停用能力。
5. 监控策略漂移、任务分布漂移、ETA 校准误差、优先级组公平性、奖励分项和异常 fallback。
6. 若业务允许抢占，需另行定义取货前抢占成本；取货后默认禁止抢占，除非具备可验证的交接流程。

## 6. 已知边界

- 当前 deadline/SLA 参数是工程默认值，需要业务数据校准。
- 当前 ETA 是确定性近似加安全裕量，不替代未来基于历史轨迹的概率 ETA 模型。
- smoke/50 回合训练只验证训练链路和契约，不足以证明收敛或优于基线；120 秒 DQN 续验进一步证明该 checkpoint 尚不满足吞吐发布门禁。
- Webots 相对 checkpoint 路径和控制台诊断问题已闭环；实验入口会在启动前拒绝不存在的 checkpoint。
- 工作区原本包含大量未提交文件和结果产物；本轮未清理、覆盖或回退用户已有改动。
