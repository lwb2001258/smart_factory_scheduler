# 高级强化学习调度算法扩展 Workflow

## 1. 实施范围

本 workflow 在现有 278 维 observation、161 维 masked robot–task 动作合同上
增加：

1. SARSA(λ)；
2. Rainbow-style DQN：Double、Dueling、PER、n-step、Distributional/Noisy
   组件必须通过消融开关独立验证；
3. A2C；
4. Discrete SAC；
5. QR-DQN。

MAPPO、QMIX 暂不加入，因为当前是中央调度器单动作决策，不是每台机器人同时
产生局部动作；CQL/IQL 暂不加入，因为现有历史结果缺少完整的 state、next_state
和 action mask。未来满足相应数据/架构前提后另开 workflow。

## 2. 不可破坏的公共合同

- observation：278；action：161；NO_OP：160；
- 所有算法必须使用相同 action mask，非法动作概率/Q 值不得进入目标；
- checkpoint 必须保存算法、环境版本、合同指纹、网络结构、seed 和训练步；
- 正式评价必须使用 manifest、零 fallback 和原生 commit 门禁；
- 旧模型兼容不能伪装成新算法结果；
- 新算法不得直接修改任务或机器人真实状态，只返回候选 assignment。

## 3. 分步实施与双重审查

### 步骤 A：公共训练基础设施

- mask-aware categorical distribution；
- n-step transition accumulator；
- prioritized replay 与 importance sampling；
- 通用 Adam、梯度裁剪、数值有限性检查；
- checkpoint schema 和注册表扩展。

Review 1：公式、shape、终止/bootstrap、mask；Review 2：数值稳定、随机性、
checkpoint 恢复、内存和性能。两轮后执行专项及全量测试。

### 步骤 B：SARSA(λ) 和 Rainbow DQN

- SARSA(λ) 使用 replacing/accumulating trace 可配置；episode 结束清 trace；
- Rainbow 默认至少启用 Double、Dueling、PER 和 n-step；distributional 与 noisy
  组件须有独立测试，不能只使用 Rainbow 名称；
- 每个组件提供消融配置和 checkpoint 元数据。

Review 1：TD target/trace/priority；Review 2：终止冲刷、mask、恢复与消融真实性。

### 步骤 C：A2C 和 Discrete SAC

- A2C：masked policy、value baseline、entropy、advantage；
- Discrete SAC：双 Q、target Q、温度参数、masked categorical policy；
- SAC 目标不得把非法动作纳入 log-sum-exp/期望。

Review 1：损失与梯度；Review 2：entropy、温度、target 更新及数值稳定。

### 步骤 D：QR-DQN

- 固定 quantile 数；
- quantile Huber loss；
- masked Double-DQN action selection；
- 推理使用 quantile 均值，可扩展风险分位策略。

Review 1：pairwise quantile loss；Review 2：shape、内存、风险推理和 checkpoint。

### 步骤 E：统一训练与部署

- 扩展 scheduler factory、CLI、训练、冻结评价和模型审计；
- A/B/C manifest 课程、held-out validation、最终 test seeds 隔离；
- 每种算法 1-episode、短更新、保存/加载/推理 smoke；
- standalone 与 Webots 使用相同 checkpoint 和 provenance。

### 步骤 F：最终比较与发布

- 最少 5 seeds，报告置信区间；
- 同一 manifest 比较 SARSA、DQN、PPO 和所有新增算法；
- fallback、超时、旧合同、缺 provenance 的结果排除；
- 输出组件消融，避免把 Rainbow 改善错误归因于单一组件。

## 4. 每一步验收模板

```text
实现范围：
Review 1（正确性）发现/修正：
专项测试：
Review 2（数值、回归、实验真实性）发现/修正：
全量测试：
Smoke checkpoint：
遗留风险：
验收：PASS / FAIL
```

