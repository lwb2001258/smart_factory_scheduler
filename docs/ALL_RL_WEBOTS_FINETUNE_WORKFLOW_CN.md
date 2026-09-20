# 全强化学习算法 Webots 微调工作流

本工作流支持 DQN、SARSA、PPO、SARSA_LAMBDA、RAINBOW_DQN、A2C、DISCRETE_SAC 和 QR_DQN。顺序固定为：模拟训练选择最佳 checkpoint、Webots 采集、离线微调、独立 seed 配对验证、满足门禁后晋升 champion。

## 强制约束

- 数据集版本为 `webots-transition-v2-current-mask`，同时保存当前动作 mask、下一动作和下一状态 mask。
- 采集结果含任何 fallback、无 RL 决策、checkpoint 哈希不一致时拒绝训练。
- `-Resume` 必须重新核对 collection manifest、base/candidate/dataset SHA256、算法、场景、时长、seed、契约和数据集版本。旧的 v1 目录不能恢复。
- collection seeds、validation seeds 和最终测试 seeds 必须互斥。
- PPO 使用采集 checkpoint 计算冻结的旧策略概率和 TD advantage，再执行 clipped importance actor-critic 更新；SARSA 系列使用轨迹中记录的真实 next action。
- 微调候选必须经过真实 Webots 配对验证；模拟指标不能触发 champion 晋升。

## 单个算法

以下命令从 CMD 执行。将 `DQN` 和 checkpoint 路径替换为目标算法即可。

```cmd
cd /d D:\code\smart_factory_scheduler
powershell.exe -NoProfile -ExecutionPolicy Bypass ^
  -File scripts\run_webots_finetune_workflow.ps1 ^
  -Algorithm DQN ^
  -BaseCheckpoint results\models\v8\dqn\best_validation.pkl ^
  -Webots "C:\Program Files\Webots\msys64\mingw64\bin\webots.exe" ^
  -Scenario C ^
  -CollectionSeeds 300001,300002,300003,300004,300005 ^
  -ValidationSeeds 310001,310002,310003,310004,310005 ^
  -Duration 1800 ^
  -OutputDir results\webots_finetune\dqn_v2 ^
  -Updates 100
```

默认 RL 推理预算为 0.5 秒。工作流仍会拒绝任何超时/fallback 结果，并对启动失败或门禁拒绝执行指定次数的完整 Webots 重试。可通过 `-RlTimeoutSeconds` 为特定机器显式调整。

`-Algorithm` 可取：`DQN`、`SARSA`、`PPO`、`SARSA_LAMBDA`、`RAINBOW_DQN`、`A2C`、`DISCRETE_SAC`、`QR_DQN`。

中断后只能用完全相同的参数恢复：

```cmd
  -Updates 100 ^
  -Resume
```

如果数据、模型或参数被修改，恢复会明确失败；请使用新输出目录重新采集。

## 一次顺序执行全部算法

```cmd
powershell.exe -NoProfile -ExecutionPolicy Bypass ^
  -File scripts\run_all_rl_webots_finetune_workflow.ps1 ^
  -DqnCheckpoint results\models\v8\dqn\best_validation.pkl ^
  -SarsaCheckpoint results\models\v8\sarsa\best_validation.json ^
  -PpoCheckpoint results\models\v8\ppo\ppo_model_best_validation.npz ^
  -SarsaLambdaCheckpoint results\models\v8\sarsa_lambda\model.json ^
  -RainbowDqnCheckpoint results\models\v8\rainbow_dqn\model.npz ^
  -A2cCheckpoint results\models\v8\a2c\model.npz ^
  -DiscreteSacCheckpoint results\models\v8\discrete_sac\model.npz ^
  -QrDqnCheckpoint results\models\v8\qr_dqn\model.npz ^
  -Webots "C:\Program Files\Webots\msys64\mingw64\bin\webots.exe" ^
  -Scenario C ^
  -CollectionSeeds 300001,300002,300003,300004,300005 ^
  -ValidationSeeds 310001,310002,310003,310004,310005 ^
  -Duration 1800 ^
  -OutputRoot results\webots_finetune\all_rl_v2 ^
  -Updates 100
```

该脚本按算法顺序执行并 fail-fast；某个算法失败后不会继续运行后续算法。修复失败原因后，使用相同参数追加 `-Resume`。

## 高级算法模拟训练的统一选模

五种高级算法现在使用相同的 `scheduler-selection-v1` 目标，在固定 held-out seeds 上周期评估并仅保存最佳 checkpoint：

```cmd
python scripts\train_advanced_rl.py ^
  --algorithm RAINBOW_DQN ^
  --episodes 5000 ^
  --validation-interval 10 ^
  --validation-seeds 204200 204201 204202 204203 204204 ^
  --checkpoint-dir results\models\v8\rainbow_dqn
```

不要根据最终测试集结果重新选择 checkpoint 或调整参数。

## 主要输出

- `collection_manifest.json`：不可变恢复依据。
- `webots_train.jsonl`：审计后的 v2 transitions。
- `candidate/finetune_report.json`：算法、更新方法、哈希和契约信息。
- `validation/`：base/candidate 的配对 Webots 结果。
- `promotion_report.json`：统一评价目标和安全门禁结果。
- `champion.*`：仅在所有晋升条件通过后生成。
