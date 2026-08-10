# PPO 与 DQN/SARSA 统一训练环境迁移说明

## 已完成修复

新 PPO 已迁移到与 DQN、SARSA 相同的 `SchedulingEnvironment`：

- observation dimension：278；
- action dimension：161；
- 动作语义：`robot_slot × task_slot + no-op`；
- 使用相同的机器人槽、任务槽和候选路线成本特征；
- 使用相同的 feasibility action mask；
- 使用 `RewardConfig` 奖励结构；
- 使用相同的 factory-grid A* 训练场景生成器；
- validation 使用 5 个 held-out seed；
- checkpoint 选择分数与 DQN/SARSA 的语义一致。

## Masked PPO 修复

每个 PPO transition 现在同时保存：

```text
state
action
reward
value
old masked log probability
done
action mask
```

更新阶段使用同一个 action mask 重新构造新策略概率，因此：

```text
old_log_prob = log(masked_old_policy[action])
new_log_prob = log(masked_new_policy[action])
```

PPO ratio、entropy 和 policy gradient 均只在合法动作上计算。旧实现将 masked old probability 与 unmasked new probability 混合的问题已经消除。

## Webots 推理

新 checkpoint 会自动加载 `PairwisePPOScheduler`：

```text
Webots snapshot
  → SchedulingEnvironment.observe()
  → pairwise action mask
  → PPO masked policy
  → argmax legal robot-task action
  → Assignment validation
  → Supervisor path planning and dispatch
```

DQN、SARSA、PPO 现在共享相同的 Webots observation/action adapter。

## 旧 checkpoint 兼容

旧模型维度：

```text
state_dim = 126
action_dim = 8
```

旧模型仍可加载，并自动使用 legacy route-aware adapter，以便复现实验结果。

新模型维度：

```text
state_dim = 278
action_dim = 161
```

新旧模型不能直接 resume，因为网络输入和输出维度发生了必要变化。新 PPO 必须从头训练。

## 新训练命令

快速训练和 Webots A/B/C 测试：

```powershell
.\scripts\run_ppo_auto_optimize_webots.ps1 `
  -Profile Quick `
  -EvaluationSeedValues 1 `
  -Duration 600
```

完整训练：

```powershell
.\scripts\run_ppo_full_webots_seed1.ps1
```

## 验证状态

已完成：

- Python 编译检查；
- 2 episode pairwise PPO smoke training；
- 278 维 observation 检查；
- 161 维 action 检查；
- action mask 训练/更新一致性检查；
- checkpoint 保存和加载；
- 新模型 `PairwisePPOScheduler` 推理；
- 旧 126×8 PPO checkpoint 兼容加载。

尚未完成：

- Full 5,000 episode 新 PPO 训练；
- 新旧 PPO 的多 seed Webots A/B/C 正式比较；
- PPO、DQN、SARSA 的同 seed 置信区间比较。
