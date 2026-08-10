<#
Train the available RL models in standalone mode, then run all configured
schedulers in headless Webots and aggregate the produced JSON metrics.

Example:
  .\scripts\run_all_algorithms_headless.ps1 -Seeds 1 -Duration 1800
#>
[CmdletBinding()]
param(
    [int]$Seeds = 1,
    [int]$Duration = 1800,
    [int]$TrainingEpisodes = 200,
    [string]$Webots = "C:\Program Files\Webots\msys64\mingw64\bin\webots.exe",
    [string]$OutputDir = "results\headless_runs"
)

$ErrorActionPreference = "Continue"
$project = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
Set-Location $project
$runStarted = Get-Date
$env:PYTHONDONTWRITEBYTECODE = "1"
$env:SMART_FACTORY_SIM_DURATION = [string]$Duration

New-Item -ItemType Directory -Force -Path $OutputDir | Out-Null
$runLog = Join-Path $OutputDir "run.log"
"Started $(Get-Date -Format o)" | Set-Content $runLog

function Invoke-Logged {
    param([string]$Label, [string]$File, [string[]]$Arguments)
    "`n===== $Label =====" | Tee-Object -FilePath $runLog -Append
    & python $File @Arguments 2>&1 | Tee-Object -FilePath $runLog -Append
    $code = $LASTEXITCODE
    "exit_code=$code" | Tee-Object -FilePath $runLog -Append
    return $code
}

Write-Host "[1/3] Training DQN and SARSA in standalone abstract environment"
Invoke-Logged "Train DQN" "scripts\train_scheduler.py" @(
    "--algorithm", "dqn", "--episodes", $TrainingEpisodes,
    "--seed", "42", "--checkpoint-dir", "results\models\headless_dqn"
) | Out-Null
Invoke-Logged "Train SARSA" "scripts\train_scheduler.py" @(
    "--algorithm", "sarsa", "--episodes", $TrainingEpisodes,
    "--seed", "42", "--checkpoint-dir", "results\models\headless_sarsa"
) | Out-Null
Invoke-Logged "Train PPO" "scripts\train_ppo.py" @(
    "--episodes", $TrainingEpisodes, "--save-dir", "results\models\headless_ppo"
) | Out-Null

if (-not (Test-Path $Webots)) {
    Write-Warning "Webots executable not found: $Webots"
    exit 2
}

$baseline = @("FCFS", "NearestNeighbour", "RoundRobin", "Greedy", "Random", "Hungarian", "Auction", "GA", "SA", "PPO_RL")
$scenarios = @("A", "B", "C")
Write-Host "[2/3] Running headless Webots: duration=$Duration seconds, seeds=$Seeds"

foreach ($scenario in $scenarios) {
    foreach ($algorithm in $baseline) {
        Remove-Item Env:MODEL_PATH -ErrorAction SilentlyContinue
        if ($algorithm -eq "PPO_RL") {
            $env:MODEL_PATH = (Resolve-Path "results\models\headless_ppo\ppo_model_final.npz").Path
        }
        Invoke-Logged "$scenario/$algorithm" "scripts\run_experiments.py" @(
            "--scenario", $scenario, "--scheduler", $algorithm,
            "--seeds", $Seeds, "--webots", $Webots
        ) | Out-Null
    }

    $env:MODEL_PATH = (Resolve-Path "results\models\headless_dqn\best_validation.pkl").Path
    Invoke-Logged "$scenario/DQN" "scripts\run_experiments.py" @(
        "--scenario", $scenario, "--scheduler", "DQN",
        "--seeds", $Seeds, "--webots", $Webots
    ) | Out-Null

    $env:MODEL_PATH = (Resolve-Path "results\models\headless_sarsa\best_validation.json").Path
    Invoke-Logged "$scenario/SARSA" "scripts\run_experiments.py" @(
        "--scenario", $scenario, "--scheduler", "SARSA",
        "--seeds", $Seeds, "--webots", $Webots
    ) | Out-Null
}
Remove-Item Env:MODEL_PATH -ErrorAction SilentlyContinue

Write-Host "[3/3] Aggregating JSON metrics"
$files = Get-ChildItem results -Filter "experiment_*_*.json" -File |
    Where-Object { $_.LastWriteTime -ge $runStarted }
$rows = @()
foreach ($file in $files) {
    try {
        $data = Get-Content $file.FullName -Raw | ConvertFrom-Json
        if (-not $data.experiment_info -or -not $data.summary_metrics) { continue }
        $i = $data.experiment_info; $m = $data.summary_metrics
        $rows += [pscustomobject]@{
            Scenario = $i.scenario; Algorithm = $i.scheduler; File = $file.Name
            Completed = [int]$m.total_tasks_completed; Generated = [int]$m.total_tasks_generated
            CompletionRate = if ([int]$m.total_tasks_generated) { [double]$m.total_tasks_completed / [double]$m.total_tasks_generated } else { 0 }
            ThroughputPerMin = [double]$m.throughput_per_minute
            AvgWaitSeconds = [double]$m.avg_waiting_time
            AvgCompletionSeconds = [double]$m.avg_task_completion_time
            P95SchedulingMs = [double]$m.scheduling_latency_p95_ms
            MinPairDistance = [double]$m.min_pair_distance
            PairDistanceViolations = [int]$m.pair_distance_violations
            Deadlocks = [int]$m.total_deadlocks
            InvalidOutputs = [int]$m.invalid_scheduler_outputs
            Fallbacks = [int]$m.scheduler_fallbacks
        }
    } catch { Write-Warning "Cannot parse $($file.Name): $_" }
}
$csv = Join-Path $OutputDir "algorithm_metrics.csv"
$rows | Sort-Object Scenario,Algorithm | Export-Csv $csv -NoTypeInformation -Encoding UTF8

$best = $rows | Group-Object Scenario | ForEach-Object {
    $_.Group | Sort-Object `
        @{Expression="PairDistanceViolations";Descending=$false}, `
        @{Expression="CompletionRate";Descending=$true}, `
        @{Expression="ThroughputPerMin";Descending=$true}, `
        @{Expression="AvgWaitSeconds";Descending=$false} |
        Select-Object -First 1
}
$best | Format-Table Scenario,Algorithm,PairDistanceViolations,MinPairDistance,CompletionRate,ThroughputPerMin,AvgWaitSeconds,Deadlocks -AutoSize |
    Out-File (Join-Path $OutputDir "best_by_scenario.txt") -Encoding UTF8

Write-Host "Metrics: $csv"
Write-Host "Best by scenario: $(Join-Path $OutputDir 'best_by_scenario.txt')"
Write-Host "Run log: $runLog"
Write-Host "Finished $(Get-Date -Format o)"
