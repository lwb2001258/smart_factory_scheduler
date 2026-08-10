<#
.SYNOPSIS
Automatically optimise PPO, promote the best validation checkpoint, and
evaluate the current route-aware PPO scheduler in Webots Scenes A/B/C.

.EXAMPLE
  .\scripts\run_ppo_auto_optimize_webots.ps1

.EXAMPLE
  .\scripts\run_ppo_auto_optimize_webots.ps1 -Profile Full -EvaluationSeedValues 1

.EXAMPLE
  .\scripts\run_ppo_auto_optimize_webots.ps1 -SkipOptimization -Duration 600

The default Quick profile is intended for a time-constrained validation. Full
performs the repository's complete PPO search and 5,000-episode champion run.
#>
[CmdletBinding()]
param(
    [ValidateSet("Quick", "Full")]
    [string]$Profile = "Quick",

    [int[]]$EvaluationSeedValues = @(1),

    [ValidateRange(60, 7200)]
    [int]$Duration = 600,

    [ValidateRange(0.0, 1.0)]
    [double]$RouteWeight = 0.50,

    [ValidateRange(1, 20)]
    [int]$TaskCandidates = 8,

    [ValidateRange(10, 1000)]
    [int]$InferenceTimeoutMilliseconds = 50,

    [string]$Webots = "C:\Program Files\Webots\msys64\mingw64\bin\webots.exe",

    [string]$OutputDirectory = "results\ppo_auto_optimized",

    [switch]$SkipOptimization,

    [switch]$StandaloneEvaluation
)

$ErrorActionPreference = "Stop"
$ProjectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
Set-Location $ProjectRoot

if (-not (Get-Command python -ErrorAction SilentlyContinue)) {
    throw "Python is not available on PATH."
}

$OutputPath = if ([System.IO.Path]::IsPathRooted($OutputDirectory)) {
    $OutputDirectory
} else {
    Join-Path $ProjectRoot $OutputDirectory
}
New-Item -ItemType Directory -Force -Path $OutputPath | Out-Null

$SearchReport = Join-Path $OutputPath "search_report.json"
$Checkpoint = Join-Path $OutputPath "best\ppo\ppo_model_best_validation.npz"

$env:PYTHONDONTWRITEBYTECODE = "1"
$env:SMART_FACTORY_SIM_DURATION = [string]$Duration
$env:PPO_ROUTE_AWARE = "1"
$env:PPO_ROUTE_WEIGHT = [string]::Format(
    [System.Globalization.CultureInfo]::InvariantCulture,
    "{0:0.00}", $RouteWeight)
$env:PPO_TASK_CANDIDATES = [string]$TaskCandidates
$env:RL_SCHEDULER_TIMEOUT_SECONDS = [string]::Format(
    [System.Globalization.CultureInfo]::InvariantCulture,
    "{0:0.000}", ($InferenceTimeoutMilliseconds / 1000.0))

Write-Host "============================================================"
Write-Host "PPO automatic optimisation + Webots A/B/C evaluation"
Write-Host "Profile: $Profile"
Write-Host "Evaluation seeds: $($EvaluationSeedValues -join ', ')"
Write-Host "Duration per scene/seed: $Duration simulation seconds"
Write-Host "Route-aware PPO: weight=$($env:PPO_ROUTE_WEIGHT), candidates=$TaskCandidates"
Write-Host "RL inference timeout: $InferenceTimeoutMilliseconds ms"
Write-Host "Output: $OutputPath"
Write-Host "============================================================"

if (-not $SkipOptimization) {
    Write-Host "[1/4] Searching PPO hyperparameters and training champion model"
    python scripts/search_hyperparameters.py `
        --profile $($Profile.ToLowerInvariant()) `
        --algorithms ppo `
        --output $OutputPath
    if ($LASTEXITCODE -ne 0) {
        throw "PPO hyperparameter search failed with exit code $LASTEXITCODE"
    }
} else {
    Write-Host "[1/4] Optimisation skipped; reusing promoted checkpoint"
}

if (-not (Test-Path -LiteralPath $SearchReport -PathType Leaf)) {
    throw "PPO search report not found: $SearchReport"
}
if (-not (Test-Path -LiteralPath $Checkpoint -PathType Leaf)) {
    throw "Promoted PPO checkpoint not found: $Checkpoint"
}
$Checkpoint = (Resolve-Path -LiteralPath $Checkpoint).Path

Write-Host "[2/4] Verifying promoted PPO checkpoint metadata"
@'
import sys
sys.path.insert(0, 'controllers/factory_supervisor')
from schedulers import create_scheduler
create_scheduler('PPO_RL', sys.argv[1], seed=1, allow_safe_fallback=False)
print('Validated PPO checkpoint:', sys.argv[1])
'@ | python - $Checkpoint
if ($LASTEXITCODE -ne 0) {
    throw "Promoted PPO checkpoint validation failed"
}

Write-Host "[3/4] Evaluating PPO on Scenes A/B/C"
$EvaluationArguments = @(
    "scripts/run_experiments.py",
    "--scenario", "A", "B", "C",
    "--scheduler", "PPO_RL",
    "--seed-values"
) + $EvaluationSeedValues + @(
    "--ppo-model", $Checkpoint
)

if ($StandaloneEvaluation) {
    $EvaluationArguments += "--standalone"
    Write-Host "Evaluation mode: standalone (explicitly requested)"
} else {
    if (-not (Test-Path -LiteralPath $Webots -PathType Leaf)) {
        throw "Webots executable not found: $Webots"
    }
    $EvaluationArguments += @("--webots", $Webots)
    Write-Host "Evaluation mode: Webots batch/fast/no-rendering"
}

python @EvaluationArguments
if ($LASTEXITCODE -ne 0) {
    throw "PPO A/B/C evaluation failed with exit code $LASTEXITCODE"
}

Write-Host "[4/4] Completed"
Write-Host "Best checkpoint: $Checkpoint"
Write-Host "Search report: $SearchReport"
Write-Host "Experiment JSON: results\experiment_[A-C]_PPO_RL_*.json"
Write-Host "Comparison report: results\comparison_report.txt"

Remove-Item Env:SMART_FACTORY_SIM_DURATION -ErrorAction SilentlyContinue
Remove-Item Env:PPO_ROUTE_AWARE -ErrorAction SilentlyContinue
Remove-Item Env:PPO_ROUTE_WEIGHT -ErrorAction SilentlyContinue
Remove-Item Env:PPO_TASK_CANDIDATES -ErrorAction SilentlyContinue
Remove-Item Env:RL_SCHEDULER_TIMEOUT_SECONDS -ErrorAction SilentlyContinue
