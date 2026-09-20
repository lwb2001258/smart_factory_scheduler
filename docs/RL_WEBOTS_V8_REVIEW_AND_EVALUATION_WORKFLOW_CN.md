# RL–Webots v8 一致性、双轮审查与公平评价工作流

本文件是后续训练、选模和论文实验的强制入口规则。若旧文档与本文件冲突，以本文件为准。

## 步骤一：训练环境与 Webots 行为代理

1. `SchedulingEnvironment` 必须通过 `WebotsBehaviorProxy` 计算任务与充电耗时。
2. 路径必须来自与运行时静态代价一致的平滑 Grid-A* 几何；代理统一处理 0.22 m/s、转向、0.35 m waypoint 容差和 Webots timestep 量化。
3. 充电必须按 `BATTERY_CHARGE_RATE` 连续计算到 `FULL_BATTERY_THRESHOLD`，禁止随机或固定时长补电。
4. 改动时间、动作、状态或奖励语义时必须升级 `RL_ENVIRONMENT_VERSION`，旧 checkpoint 不得静默加载。
5. 每次修改必须完成两轮 review：第一轮检查物理/状态机契约，第二轮检查 checkpoint 兼容性和边界事件；两轮均运行针对性测试。

行为代理仍不是刚体物理替代品。动态避障、碰撞、通信失败和重规划只能通过最终 Webots 配对实验验证，训练结果不得直接作为论文性能结论。

## 步骤二：统一算法优劣目标

1. DQN、SARSA、PPO 和元启发式调参必须调用 `evaluation_objective.algorithm_selection_score`。
2. 完成率越高越好；完成时间、等待时间、makespan、距离、非法动作、原生失败和延迟均越低越好；reward 只用于小权重破同分。
3. 训练产物必须记录 `selection_objective` 版本与字段，禁止在单个算法脚本内另写评分公式。
4. 每次调整权重必须完成两轮 review，并用方向性测试和至少一次训练 smoke test 验证。

## 步骤三：Webots 公平比较门禁

1. 同一场景下所有算法必须拥有完全相同的 seed 集合。
2. 对每个 seed，`run_mode`、manifest fingerprint/version 和 `sim_duration` 必须一致，否则停止生成比较报告。
3. standalone 与 Webots 结果禁止混排；论文图表仅接收 provenance 完整的 Webots measured 数据。
4. 重复运行按最新 timestamp 去重，然后仅对所有算法共有的配对 seed 求均值；禁止取单次最大完成数作为算法代表值。
5. 报告必须显示 paired seeds 和 run mode。任何缺失、重复或混源数据都应作为硬错误处理。
6. 每次修改聚合逻辑必须完成两轮 review，并分别测试接受公平 cohort 与拒绝不公平 cohort。

## 发布前验证

依次执行：针对性测试、脚本 CLI/smoke test、全量 `pytest -q`、`compileall`、`git diff --check`。Webots 可执行文件存在时，还必须用至少两个固定 seed 做真实配对回放；若环境没有 Webots，报告中必须明确标记该项未执行，不得用 standalone 代替。

## Webots 微调闭环

当前正式支持全部八种 RL 算法。v2 transition 保存当前动作 mask 和真实 `next_action`；PPO 由采集 checkpoint 重建冻结的旧策略概率并执行 clipped importance actor-critic 更新。所有候选仍必须通过独立 Webots 配对验证才能晋升。

### 1. 准备 v8 基础模型

```powershell
python scripts/train_scheduler.py --algorithm dqn --episodes 5000 `
  --checkpoint-dir results/models/v8/dqn --evaluation-interval 50
```

环境版本变化后必须重新训练；v7 或更旧 checkpoint 不得用于 v8 数据采集。

### 2. 执行完整 workflow

从 CMD 调用 PowerShell 脚本时，可将 seed 写成 `300001,300002,300003`；脚本会显式拆分和校验。直接在 PowerShell 中调用时，也兼容字符串数组。

```powershell
powershell -ExecutionPolicy Bypass -File scripts/run_webots_finetune_workflow.ps1 `
  -BaseCheckpoint results/models/v8/dqn/best_validation.pkl `
  -Webots "C:\Program Files\Webots\msys64\mingw64\bin\webots.exe" `
  -Scenario C `
  -CollectionSeeds 300001,300002,300003 `
  -ValidationSeeds 310001,310002,310003 `
  -Duration 1800 `
  -OutputDir results/webots_finetune/dqn_v1 `
  -Updates 100
```

执行顺序为：

1. 使用 base checkpoint 和 collection seeds 运行真实 Webots，并仅记录成功提交的 state/action/mask。
2. 审计环境版本、contract fingerprint、动作维度、mask 和 decision ID。
3. 将 event ledger 按 decision ID 归因，生成带实际时间折扣的 `webots-transition-v1` JSONL。
4. 以低学习率和近端约束生成 `candidate.pkl`；基础 checkpoint 永不覆盖。
5. 在完全独立的 validation seeds 上分别运行 base 和 candidate。
6. 检查 fallback、非法输出、未授权路线写入、deadline、安全距离和完成率硬门禁。
7. 只有统一评分提高，并且吞吐提高至少 2%或完成时间降低至少 5%时，才复制为 `champion.pkl`。

主要产物：

- `webots_train.jsonl`：审计后的 Webots transition；
- `candidate/finetune_report.json`：基础/候选 hash、更新数和 loss；
- `candidate/candidate.pkl`：待验证模型；
- `promotion_report.json`：逐项晋级结论；
- `champion.pkl`：仅在全部门禁通过后产生。

### 3. 数据与实验约束

- collection、validation、最终 test seed 必须互斥；
- validation/test 运行设置 `WEBOTS_RL_COLLECT=0`，不产生训练数据；
- 短时 smoke 只能证明流程可执行，正式晋级建议至少 10 个配对 validation seed；
- 动态碰撞、异步多机器人延迟奖励仍由 Webots 实测决定，不得从抽象训练指标推断；
- 每次修改采集、归因、微调或晋级模块，都要分别完成两轮 review 和两轮针对性测试。
