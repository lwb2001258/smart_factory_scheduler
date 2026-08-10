# Windows 下全调度算法训练与 Webots 评估命令

本文给出本项目在 Windows PowerShell 中重新训练 PPO、DQN、SARSA，并以相同随机种子在 Webots 的 A、B、C 场景中评估全部调度算法的可执行命令。

## 1. 评估范围

项目当前支持 12 种调度算法：

```text
FCFS
NearestNeighbour
RoundRobin
Greedy
Random
Hungarian
Auction
GA
SA
PPO_RL
DQN
SARSA
```

其中 PPO、DQN 和 SARSA 需要先训练；其余算法无需训练。

三个 Webots 场景为：

| 场景 | 机器人数量 | 平均任务间隔 |
|---|---:|---:|
| A | 3 | 30 秒 |
| B | 5 | 15 秒 |
| C | 8 | 8 秒 |

## 2. 推荐的一键运行命令

项目已经提供自动训练、评估和汇总脚本。打开 Windows PowerShell，执行：

```powershell
Set-Location "C:\Users\lwb10\OneDrive\Desktop\smart_factory_webots_final"

$Webots = "C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"

if (-not (Test-Path $Webots)) {
    throw "找不到 Webots：$Webots"
}

python -m pip install -r requirements.txt

powershell -ExecutionPolicy Bypass -File ".\scripts\run_all_algorithms_headless.ps1" `
  -Seeds 1 `
  -Duration 1800 `
  -TrainingEpisodes 1500 `
  -Webots $Webots `
  -OutputDir "results\headless_runs_seed42"
```

参数说明：

- `-Seeds 1` 表示取项目预定义种子列表中的第一个种子，即 `42`，不是 seed 值为 1。
- 所有算法都在相同 seed 42 下运行，保证任务生成和初始随机条件可比。
- `-Duration 1800` 表示每次运行 1800 个仿真秒。
- `-TrainingEpisodes 5000` 表示重新训练三个强化学习调度器 5000 回合。
- Webots 以 `--batch --no-rendering --mode=fast` 运行。

该命令将执行：

```text
3 个训练任务
+ 12 种算法 × 3 个场景 × 1 个 seed
= 3 个训练任务 + 36 次 Webots 仿真
```

## 3. 环境准备

```powershell
Set-Location "C:\Users\lwb10\OneDrive\Desktop\smart_factory_webots_final"

$env:PYTHONDONTWRITEBYTECODE = "1"
$env:SMART_FACTORY_SIM_DURATION = "1800"
$Webots = "C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"

if (-not (Test-Path $Webots)) {
    throw "找不到 Webots：$Webots"
}

python --version
python -m pip install -r requirements.txt
```

如果 Webots 不在默认位置，可搜索：

```powershell
Get-ChildItem "C:\Program Files" -Recurse -Filter "webots.exe" -ErrorAction SilentlyContinue |
    Select-Object -ExpandProperty FullName
```

## 4. 分步骤重新训练 RL 算法

### 4.1 DQN standalone 训练

```powershell
python scripts\train_scheduler.py `
  --algorithm dqn `
  --episodes 5000 `
  --seed 42 `
  --evaluation-interval 100 `
  --checkpoint-dir results\models\seed42_dqn

if ($LASTEXITCODE -ne 0) {
    throw "DQN 训练失败，退出码：$LASTEXITCODE"
}
```

输出文件：

```text
results\models\seed42_dqn\best_validation.pkl
results\models\seed42_dqn\latest.pkl
results\models\seed42_dqn\training_metrics.json
```

Webots 正式评估应使用 `best_validation.pkl`。

### 4.2 SARSA standalone 训练与 Q-table 生成

```powershell
python scripts\train_scheduler.py `
  --algorithm sarsa `
  --episodes 5000 `
  --seed 42 `
  --evaluation-interval 100 `
  --checkpoint-dir results\models\seed42_sarsa

if ($LASTEXITCODE -ne 0) {
    throw "SARSA 训练失败，退出码：$LASTEXITCODE"
}
```

输出文件：

```text
results\models\seed42_sarsa\best_validation.json
results\models\seed42_sarsa\latest.json
results\models\seed42_sarsa\training_metrics.json
```

`best_validation.json` 保存用于推理的 SARSA Q-table 和模型元数据。

### 4.3 PPO standalone 训练

```powershell
python scripts\train_ppo.py `
  --episodes 5000 `
  --save-dir results\models\seed42_ppo

if ($LASTEXITCODE -ne 0) {
    throw "PPO 训练失败，退出码：$LASTEXITCODE"
}
```

输出文件：

```text
results\models\seed42_ppo\ppo_model_final.npz
results\models\seed42_ppo\training_history.json
```

## 5. 冻结模型 standalone 评估

模型选择完成后，应关闭探索并在测试种子域评估。

### 5.1 DQN

```powershell
python scripts\evaluate_scheduler.py `
  --algorithm dqn `
  --checkpoint results\models\seed42_dqn\best_validation.pkl `
  --split test `
  --seeds 20
```

### 5.2 SARSA

```powershell
python scripts\evaluate_scheduler.py `
  --algorithm sarsa `
  --checkpoint results\models\seed42_sarsa\best_validation.json `
  --split test `
  --seeds 20
```

### 5.3 PPO

```powershell
$env:SMART_FACTORY_SIM_DURATION = "1800"
$env:MODEL_PATH = (Resolve-Path "results\models\seed42_ppo\ppo_model_final.npz").Path

python scripts\run_experiments.py `
  --standalone `
  --scenario A B C `
  --scheduler PPO_RL `
  --seeds 1

if ($LASTEXITCODE -ne 0) {
    throw "PPO standalone 评估失败"
}

Remove-Item Env:MODEL_PATH -ErrorAction SilentlyContinue
```

`evaluate_scheduler.py --split test` 使用独立测试种子域检查泛化能力；后续 Webots 控制变量实验仍统一使用 seed 42。

## 6. Webots 基线与优化算法评估

以下命令在 A、B、C 场景中依次运行全部不需要模型的算法：

```powershell
$BaselineAlgorithms = @(
    "FCFS",
    "NearestNeighbour",
    "RoundRobin",
    "Greedy",
    "Random",
    "Hungarian",
    "Auction",
    "GA",
    "SA"
)

foreach ($Scenario in @("A", "B", "C")) {
    foreach ($Algorithm in $BaselineAlgorithms) {
        Remove-Item Env:MODEL_PATH -ErrorAction SilentlyContinue

        Write-Host "运行 Scenario=$Scenario Algorithm=$Algorithm Seed=42"

        python scripts\run_experiments.py `
          --scenario $Scenario `
          --scheduler $Algorithm `
          --seeds 1 `
          --webots $Webots

        if ($LASTEXITCODE -ne 0) {
            throw "Webots 评估失败：Scenario=$Scenario Algorithm=$Algorithm"
        }
    }
}
```

项目中的预定义随机种子是：

```text
42, 123, 456, 789, 1024
```

因此 `--seeds 1` 固定使用 seed 42；`--seeds 5` 会依次使用以上五个种子。

## 7. Webots PPO 评估

```powershell
$env:MODEL_PATH = (Resolve-Path "results\models\seed42_ppo\ppo_model_final.npz").Path

python scripts\run_experiments.py `
  --scenario A B C `
  --scheduler PPO_RL `
  --seeds 1 `
  --webots $Webots

if ($LASTEXITCODE -ne 0) {
    throw "PPO Webots 评估失败"
}

Remove-Item Env:MODEL_PATH -ErrorAction SilentlyContinue
```

## 8. Webots DQN 评估

```powershell
$env:MODEL_PATH = (Resolve-Path "results\models\seed42_dqn\best_validation.pkl").Path

python scripts\run_experiments.py `
  --scenario A B C `
  --scheduler DQN `
  --seeds 1 `
  --webots $Webots

if ($LASTEXITCODE -ne 0) {
    throw "DQN Webots 评估失败"
}

Remove-Item Env:MODEL_PATH -ErrorAction SilentlyContinue
```

## 9. Webots SARSA 评估

```powershell
$env:MODEL_PATH = (Resolve-Path "results\models\seed42_sarsa\best_validation.json").Path

python scripts\run_experiments.py `
  --scenario A B C `
  --scheduler SARSA `
  --seeds 1 `
  --webots $Webots

if ($LASTEXITCODE -ne 0) {
    throw "SARSA Webots 评估失败"
}

Remove-Item Env:MODEL_PATH -ErrorAction SilentlyContinue
```

## 10. 结果文件

每次正常达到仿真时长的 Webots 运行会生成：

```text
results\experiment_<场景>_<算法>_<时间戳>.json
```

例如：

```text
results\experiment_A_FCFS_20260802_120000.json
results\experiment_A_PPO_RL_20260802_130000.json
results\experiment_A_DQN_20260802_140000.json
```

一键脚本还会生成：

```text
results\headless_runs_seed42\algorithm_metrics.csv
results\headless_runs_seed42\best_by_scenario.txt
results\headless_runs_seed42\run.log
```

如果 Webots 被关闭、重置或因墙钟超时提前退出，Supervisor 不会把短运行保存成有效实验结果。因此，应检查每种“场景—算法”组合是否确实存在新的 JSON 文件。

## 11. 查看和排序结果

### 11.1 查看完整 CSV

```powershell
Import-Csv "results\headless_runs_seed42\algorithm_metrics.csv" |
    Format-Table -AutoSize
```

### 11.2 按项目规则排序

```powershell
Import-Csv "results\headless_runs_seed42\algorithm_metrics.csv" |
    Sort-Object Scenario,
        @{Expression={[int]$_.PairDistanceViolations}; Ascending=$true},
        @{Expression={[double]$_.CompletionRate}; Descending=$true},
        @{Expression={[double]$_.ThroughputPerMin}; Descending=$true},
        @{Expression={[double]$_.AvgWaitSeconds}; Ascending=$true} |
    Format-Table Scenario, Algorithm, Completed, CompletionRate,
        ThroughputPerMin, AvgWaitSeconds, AvgCompletionSeconds,
        PairDistanceViolations, MinPairDistance, Deadlocks,
        P95SchedulingMs, InvalidOutputs, Fallbacks -AutoSize
```

### 11.3 查看每个场景的最佳算法

```powershell
Get-Content "results\headless_runs_seed42\best_by_scenario.txt"
```

### 11.4 检查错误和模型回退

```powershell
Select-String `
  -Path "results\headless_runs_seed42\run.log" `
  -Pattern "ERROR|failed|timeout|fallback|unavailable"
```

## 12. 优劣判定原则

当前自动汇总按以下优先级选出每个场景的最佳算法：

```text
1. PairDistanceViolations 越少越好
2. CompletionRate 越高越好
3. ThroughputPerMin 越高越好
4. AvgWaitSeconds 越低越好
```

还应同时检查：

- `MinPairDistance < 0.5`：存在机器人安全距离违规。
- `Fallbacks > 0`：RL 推理发生回退，结果不能完全归功于 RL 模型。
- `InvalidOutputs > 0`：调度器曾输出无效分配。
- `Deadlocks` 或 `total_replans` 较高：算法可能造成严重拥堵。
- `P95SchedulingMs` 较高：实时计算性能较差。
- `workload_balance_cv` 较低：机器人任务分配更加均衡。
- `total_distance_all_robots` 较低：整体运输路径通常更经济。

安全性应优先于吞吐量，不能仅根据完成任务数量判断算法优劣。

## 13. 正式五种子实验

完成 seed 42 的控制变量检查后，正式报告建议使用全部五个相同种子：

```powershell
powershell -ExecutionPolicy Bypass -File ".\scripts\run_all_algorithms_headless.ps1" `
  -Seeds 5 `
  -Duration 1800 `
  -TrainingEpisodes 5000 `
  -Webots $Webots `
  -OutputDir "results\headless_runs_five_seeds"
```

该实验执行：

```text
12 种算法 × 3 个场景 × 5 个种子 = 180 次 Webots 仿真
```

正式分析应按场景和算法汇总均值、标准差、最差安全距离、失败次数和回退次数，而不是只选择单次最好结果。

## 14. 建议执行顺序

```text
环境检查
  -> DQN/SARSA/PPO 重新训练
  -> 冻结模型 standalone 测试
  -> seed 42 的 36 次 Webots 控制变量实验
  -> 检查日志、JSON 数量及 RL 回退
  -> 查看 CSV 与各场景最佳算法
  -> 通过后再执行五种子正式实验
```

训练与测试种子应保持概念分离：训练 seed 用于模型学习，Webots 中“同一 seed 比较”用于保证不同调度器面对相同任务序列和初始随机条件。
