# 高级强化学习调度算法验收记录

## 最终结论

本 workflow 已完成并通过验收。项目新增可独立训练、保存、审计和部署的
`SARSA_LAMBDA`、`RAINBOW_DQN`、`A2C`、`DISCRETE_SAC`、`QR_DQN`。
所有算法共享 278 维 observation、161 维 masked robot-task pair 动作以及动作 160 的
`NO_OP` 合同；正式推理严格加载 checkpoint，失败时不会伪装成原算法结果。

## 分步双审查与测试

### A. 公共训练设施

- 实现：masked softmax、Adam/梯度裁剪、n-step accumulator、PER/importance sampling。
- Review 1：修正 terminal n-step 的 bootstrap discount，终止转换强制为 0。
- 测试 1：mask、数值平移、terminal flush、PER 权重和 Adam 非有限梯度检查。
- Review 2：检查环形覆盖、priority 下限、shape、随机 seed 和容量配置。
- 测试 2：与全部高级算法公共回归联合运行。
- 结论：PASS。

### B. SARSA(λ) 与 Rainbow DQN

- SARSA(λ)：on-policy TD、replacing/accumulating trace、episode 清理、epsilon 衰减。
- Rainbow：Double、Dueling、PER、3-step、C51、Noisy 六个组件均实际参与算法路径；关闭任一
  声明组件会拒绝以 `RAINBOW_DQN` 启动。
- Review 1：发现新增代码插入点破坏 SARSA 类边界，已修复；复核 TD target、C51 投影、
  Dueling 反向梯度和 masked Double 选择。
- 测试 1：SARSA trace 传播/持久化；Rainbow 组件开关、分布归一化、n-step flush、PER 更新。
- Review 2：复核终止 bootstrap、Noisy 参数、checkpoint 组件元数据和非法动作隔离。
- 测试 2：专项 19 项联合测试通过。
- 结论：PASS。

### C. A2C 与 Discrete SAC

- A2C：masked categorical actor、value baseline、advantage、entropy regularization。
- Discrete SAC：masked policy、双 Q、双 target Q、soft value 与 Polyak 更新。
- Review 1：复核 policy/value gradient 和 terminal target。
- 测试 1：非法动作概率为零、terminal 不 bootstrap、save/load roundtrip。
- Review 2：确认 SAC entropy expectation 不含非法动作，双 Q 取最小值，更新保持有限。
- 测试 2：专项及联合回归通过。
- 结论：PASS。

### D. QR-DQN

- 实现：固定 quantiles、pairwise quantile Huber gradient、masked Double-DQN 选择、target 更新。
- Review 1：复核 pairwise delta 的符号、tau 权重和均值推理。
- 测试 1：terminal target、非法高 Q 动作隔离和数值有限性。
- Review 2：复核 278×161×32 参数 shape、checkpoint 架构恢复和内存规模。
- 测试 2：save/load 与注册审计联合通过。
- 结论：PASS。

### E. 训练、部署、实验与审计

- 增加统一训练入口 `scripts/train_advanced_rl.py`。
- scheduler factory、Webots/standalone 共用部署适配器、模型注册表、审计 CLI 和实验 CLI 已接入。
- Review 1：发现实验 CLI 合并模型路径的字典语法错误，已修复并加入编译检查。
- 测试 1：5 种算法各 1 episode 训练、保存、加载和 checkpoint SHA-256/合同审计通过。
- Review 2：检查正式入口的 strict checkpoint、原生提交、provenance 与 fallback 污染门禁。
- 测试 2：`run_experiments --standalone` 对 5 种算法全部通过，原生策略提交大于 0、fallback 为 0。
- 结论：PASS。

### F. 最终回归

- 第一轮：高级算法/注册/工厂专项 `27 passed`。
- 第二轮：全项目 `172 passed, 4 subtests passed`。
- 静态检查：Python 编译通过，`git diff --check` 无空白错误。
- smoke 产物：`results/advanced_rl_workflow_smoke/<ALGORITHM>/`。
- 5 秒场景 A smoke 吞吐为 0 是因为短窗口不足以完成首个运输任务；原生调度提交和零 fallback
  已通过机器门禁，不能将此 smoke 当成算法性能比较。

## 尚需长时间实验验证的事项

实现验收不等于性能优于现有算法。正式论文/生产结论仍须使用相同 manifest、至少 5 个独立 seed、
完整 A/B/C 时长、置信区间和 Rainbow 消融；checkpoint 只能在 validation 集选择，test 集只能在
模型冻结后使用。Webots 长时物理运行受本机速度影响，必须保留相同 checkpoint SHA-256 和
manifest fingerprint 才能与 standalone 结果配对比较。
