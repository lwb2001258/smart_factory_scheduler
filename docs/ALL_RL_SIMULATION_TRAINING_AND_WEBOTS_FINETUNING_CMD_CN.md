# 全部强化学习算法模拟训练与 Webots 调优 CMD 指南

本文档中的命令面向 Windows CMD，不是 PowerShell。所有命令均从项目根目录执行。

## 1. 初始化环境

```cmd
cd /d D:\code\smart_factory_scheduler
set PYTHONDONTWRITEBYTECODE=1
set WEBOTS_EXE=C:\Program Files\Webots\msys64\mingw64\bin\webots.exe
"%WEBOTS_EXE%" --version
```

当前已验证的 Webots 版本为 R2023b。

v8 修改了训练动力学和 contract fingerprint。v7 或更旧 checkpoint 不能继续用于 v8 正式训练和评估，必须重新训练。

## 2. 全部强化学习算法模拟训练

### 2.1 DQN

```cmd
python scripts\train_scheduler.py ^
  --algorithm dqn ^
  --episodes 5000 ^
  --seed 42 ^
  --checkpoint-dir results\models\v8\dqn ^
  --evaluation-interval 50 ^
  --lr 0.0001 ^
  --gamma 0.99 ^
  --hidden-size 256 ^
  --batch-size 128 ^
  --replay-capacity 100000 ^
  --warmup-steps 5000 ^
  --target-update 500 ^
  --epsilon-start 1.0 ^
  --epsilon-end 0.02 ^
  --epsilon-decay-steps 50000
```

正式模型：`results\models\v8\dqn\best_validation.pkl`。

### 2.2 SARSA

```cmd
python scripts\train_scheduler.py ^
  --algorithm sarsa ^
  --episodes 10000 ^
  --seed 42 ^
  --checkpoint-dir results\models\v8\sarsa ^
  --evaluation-interval 50 ^
  --lr 0.05 ^
  --gamma 0.99 ^
  --epsilon-start 1.0 ^
  --epsilon-end 0.02 ^
  --epsilon-decay 0.9995
```

正式模型：`results\models\v8\sarsa\best_validation.json`。

### 2.3 PPO

```cmd
python scripts\train_ppo.py ^
  --episodes 5000 ^
  --seed 42 ^
  --save-dir results\models\v8\ppo ^
  --lr 0.0003 ^
  --gamma 0.99 ^
  --gae-lambda 0.95 ^
  --clip 0.2 ^
  --entropy 0.02 ^
  --value-coeff 0.5 ^
  --epochs 8 ^
  --batch-size 256 ^
  --buffer-size 2048 ^
  --hidden-size 256
```

正式模型：`results\models\v8\ppo\ppo_model_best_validation.npz`。

### 2.4 SARSA(λ)

```cmd
python scripts\train_advanced_rl.py ^
  --algorithm SARSA_LAMBDA ^
  --episodes 5000 ^
  --seed 42 ^
  --batch-size 32 ^
  --checkpoint-dir results\models\v8\sarsa_lambda
```

模型：`results\models\v8\sarsa_lambda\model.json`。

### 2.5 Rainbow DQN

```cmd
python scripts\train_advanced_rl.py ^
  --algorithm RAINBOW_DQN ^
  --episodes 5000 ^
  --seed 42 ^
  --batch-size 64 ^
  --checkpoint-dir results\models\v8\rainbow_dqn
```

模型：`results\models\v8\rainbow_dqn\model.npz`。

### 2.6 A2C

```cmd
python scripts\train_advanced_rl.py ^
  --algorithm A2C ^
  --episodes 5000 ^
  --seed 42 ^
  --batch-size 32 ^
  --checkpoint-dir results\models\v8\a2c
```

模型：`results\models\v8\a2c\model.npz`。

### 2.7 Discrete SAC

```cmd
python scripts\train_advanced_rl.py ^
  --algorithm DISCRETE_SAC ^
  --episodes 5000 ^
  --seed 42 ^
  --batch-size 64 ^
  --checkpoint-dir results\models\v8\discrete_sac
```

模型：`results\models\v8\discrete_sac\model.npz`。

### 2.8 QR-DQN

```cmd
python scripts\train_advanced_rl.py ^
  --algorithm QR_DQN ^
  --episodes 5000 ^
  --seed 42 ^
  --batch-size 64 ^
  --checkpoint-dir results\models\v8\qr_dqn
```

模型：`results\models\v8\qr_dqn\model.npz`。

## 3. 模型审计

```cmd
python scripts\audit_rl_models.py --algorithm DQN --checkpoint results\models\v8\dqn\best_validation.pkl
python scripts\audit_rl_models.py --algorithm SARSA --checkpoint results\models\v8\sarsa\best_validation.json
python scripts\audit_rl_models.py --algorithm PPO_RL --checkpoint results\models\v8\ppo\ppo_model_best_validation.npz
python scripts\audit_rl_models.py --algorithm SARSA_LAMBDA --checkpoint results\models\v8\sarsa_lambda\model.json
python scripts\audit_rl_models.py --algorithm RAINBOW_DQN --checkpoint results\models\v8\rainbow_dqn\model.npz
python scripts\audit_rl_models.py --algorithm A2C --checkpoint results\models\v8\a2c\model.npz
python scripts\audit_rl_models.py --algorithm DISCRETE_SAC --checkpoint results\models\v8\discrete_sac\model.npz
python scripts\audit_rl_models.py --algorithm QR_DQN --checkpoint results\models\v8\qr_dqn\model.npz
```

如果当前审计脚本不接受某个算法名称，先执行：

```cmd
python scripts\audit_rl_models.py --help
```

任何审计失败的模型都不能进入正式 Webots 实验。

## 4. 全部 RL 算法 Webots 自动调优

当前 Webots 微调闭环支持 DQN、SARSA、PPO、SARSA_LAMBDA、RAINBOW_DQN、A2C、DISCRETE_SAC 和 QR_DQN；通过 `-Algorithm` 选择算法。

从 CMD 调用时，seed 列表使用逗号分隔的单个参数值；脚本会负责拆分并转换为整数。不要在逗号后加入额外引号。

### 4.1 短流程验证

```cmd
powershell.exe -NoProfile -ExecutionPolicy Bypass ^
  -File scripts\run_webots_finetune_workflow.ps1 ^
  -BaseCheckpoint results\models\v8\dqn\best_validation.pkl ^
  -Webots "%WEBOTS_EXE%" ^
  -Scenario A ^
  -CollectionSeeds 300001,300002 ^
  -ValidationSeeds 310001,310002 ^
  -Duration 120 ^
  -OutputDir results\webots_finetune\dqn_quick ^
  -Updates 20
```

### 4.2 正式调优

```cmd
powershell.exe -NoProfile -ExecutionPolicy Bypass ^
  -File scripts\run_webots_finetune_workflow.ps1 ^
  -BaseCheckpoint results\models\v8\dqn\best_validation.pkl ^
  -Webots "%WEBOTS_EXE%" ^
  -Scenario C ^
  -CollectionSeeds 300001,300002,300003,300004,300005 ^
  -ValidationSeeds 310001,310002,310003,310004,310005 ^
  -Duration 1800 ^
  -OutputDir results\webots_finetune\dqn_full ^
  -Updates 100
```

主要输出：

- `webots_train.jsonl`：经过审计的 Webots transition；
- `candidate\candidate.pkl`：待验证候选模型；
- `candidate\finetune_report.json`：微调数据及 checkpoint hash；
- `promotion_report.json`：配对验证和晋级结论；
- `champion.pkl`：仅在全部门禁通过时产生。

没有产生 `champion.pkl` 不代表程序执行失败，而是 candidate 没有达到性能或安全晋级标准。

如果采集或验证中途失败，使用完全相同的参数并追加 `-Resume`。脚本会复用已经生成的 dataset、candidate 和逐 seed 验证快照：

```cmd
  -Updates 100 ^
  -Resume
```

脚本默认对临时 Webots 启动失败重试 2 次，也可以通过 `-WebotsRetries 3` 调整。具体错误会写入 `workflow_error.json`，完整控制台输出会写入 `workflow_console.log`。

workflow 默认将 RL 推理时限设为 0.5 秒，以覆盖 Windows/Webots 中的 Grid-A* observation 开销；可以通过 `-RlTimeoutSeconds` 调整。任何包含 fallback 的采集或验证结果都会被拒绝并按重试策略重新运行。

## 5. 全部算法 Webots 原生评估

模拟训练和模型审计全部通过后执行：

```cmd
set SMART_FACTORY_SIM_DURATION=1800
set WEBOTS_RL_COLLECT=0

python scripts\run_experiments.py ^
  --scenario A B C ^
  --scheduler SARSA DQN PPO_RL SARSA_LAMBDA RAINBOW_DQN A2C DISCRETE_SAC QR_DQN ^
  --seed-values 320001 320002 320003 320004 320005 ^
  --webots "%WEBOTS_EXE%" ^
  --sarsa-model results\models\v8\sarsa\best_validation.json ^
  --dqn-model results\models\v8\dqn\best_validation.pkl ^
  --ppo-model results\models\v8\ppo\ppo_model_best_validation.npz ^
  --advanced-model SARSA_LAMBDA=results\models\v8\sarsa_lambda\model.json ^
  --advanced-model RAINBOW_DQN=results\models\v8\rainbow_dqn\model.npz ^
  --advanced-model A2C=results\models\v8\a2c\model.npz ^
  --advanced-model DISCRETE_SAC=results\models\v8\discrete_sac\model.npz ^
  --advanced-model QR_DQN=results\models\v8\qr_dqn\model.npz
```

`320xxx` 是最终测试 seed。不要根据这些结果继续修改超参数，否则会造成测试集泄漏。

## 6. 为其他算法采集 Webots 数据

PPO、SARSA、SARSA(λ)、Rainbow DQN、A2C、Discrete SAC 和 QR-DQN 当前可以进行 Webots 评估与轨迹采集，但尚未开放自动梯度微调。

```cmd
set WEBOTS_RL_COLLECT=1
set SMART_FACTORY_SIM_DURATION=1800
```

PPO 采集示例：

```cmd
python scripts\run_experiments.py ^
  --scenario C ^
  --scheduler PPO_RL ^
  --seed-values 300101 300102 300103 ^
  --webots "%WEBOTS_EXE%" ^
  --ppo-model results\models\v8\ppo\ppo_model_best_validation.npz
```

Rainbow DQN 采集示例：

```cmd
python scripts\run_experiments.py ^
  --scenario C ^
  --scheduler RAINBOW_DQN ^
  --seed-values 300201 300202 300203 ^
  --webots "%WEBOTS_EXE%" ^
  --advanced-model RAINBOW_DQN=results\models\v8\rainbow_dqn\model.npz
```

将实验 JSON 转换成数据集：

```cmd
python scripts\build_webots_dataset.py ^
  --input results\experiment_C_PPO_RL_时间戳1.json results\experiment_C_PPO_RL_时间戳2.json ^
  --output results\webots_finetune\ppo\webots_train.jsonl
```

不要把其他算法的数据交给 DQN 微调器。当前限制如下：

- PPO 仍需采样时的 log-probability、value 和完整 on-policy trajectory；
- SARSA/SARSA(λ) 仍需保存实际 next action；
- A2C、Discrete SAC、QR-DQN、Rainbow DQN 需要各自的更新适配器；
- `finetune_from_webots.py` 支持全部八种 RL 算法，并按 checkpoint 算法和 v2 数据契约严格校验。

## 7. 推荐执行顺序

1. 训练 DQN 和 SARSA，确认 v8 基础训练正常；
2. 训练 PPO；
3. 训练五种 advanced RL；
4. 审计全部 checkpoint；
5. 用 120 秒短时 Webots 对全部模型进行启动验证；
6. 依次执行全部 RL 算法的 Webots 微调；
7. 用独立 seed 对全部算法进行 1800 秒 Webots 配对评估；
8. 在其余算法的微调适配器完成前，只采集其 Webots 数据，不执行方法学不正确的梯度更新。

正式实验应至少使用 5 个配对 seed；论文结果建议使用 10 个或更多配对 seed，并报告均值、中位数、标准差、置信区间及配对显著性检验。
