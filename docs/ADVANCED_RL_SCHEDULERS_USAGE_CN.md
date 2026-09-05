# 高级强化学习调度算法使用说明

支持的名称：`SARSA_LAMBDA`、`RAINBOW_DQN`、`A2C`、`DISCRETE_SAC`、`QR_DQN`。

训练示例：

```powershell
python scripts/train_advanced_rl.py --algorithm A2C --episodes 200 `
  --checkpoint-dir results/models/A2C
```

其他算法只需替换 `--algorithm`。SARSA(λ) 输出 `model.json`，其余输出 `model.npz`。

模型审计：

```powershell
python scripts/audit_rl_models.py --algorithm A2C `
  --checkpoint results/models/A2C/model.npz
```

standalone 正式入口：

```powershell
python scripts/run_experiments.py --standalone --scenario A B C `
  --scheduler A2C --seed-values 42 123 456 789 1024 `
  --advanced-model A2C=results/models/A2C/model.npz
```

Webots 使用同一命令但去掉 `--standalone`，并通过 `--webots` 指定可执行文件（若不在 PATH）。
正式 RL 实验默认不允许缺失模型或 fallback；`--allow-rl-fallback` 只用于诊断，其结果不得作为
原生算法成绩。

训练环境和 Webots 部署共用 `RL_SCHEDULING_CONTRACT`、相同 observation 编码、action mask、
assignment 解码和 checkpoint fingerprint。训练脚本只在私有 abstract 状态上执行动作；部署适配器
只返回候选 assignment，不直接修改 Webots 的任务或机器人真实状态。
