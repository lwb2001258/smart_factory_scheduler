<#
Train PPO and evaluate the same checkpoint.

Standalone example:
  .\scripts\run_ppo_train_evaluate.ps1 -Episodes 500 -Seeds 5

Webots example:
  .\scripts\run_ppo_train_evaluate.ps1 -Episodes 500 -Seeds 1 -RunWebots -Duration 1800
#>
[CmdletBinding()]
param(
    [int]$Episodes = 500,
    [int]$Seeds = 5,
    [int]$Duration = 1800,
    [switch]$RunWebots,
    [string]$Webots = "C:\Program Files\Webots\msys64\mingw64\bin\webots.exe",
    [string]$CheckpointDir = "results\models\ppo_run"
)

$ErrorActionPreference = "Stop"
Set-Location (Resolve-Path (Join-Path $PSScriptRoot ".."))
$env:PYTHONDONTWRITEBYTECODE = "1"
New-Item -ItemType Directory -Force -Path $CheckpointDir | Out-Null

Write-Host "[1/3] Training PPO in the project's standalone trainer"
python scripts/train_ppo.py `
  --episodes $Episodes `
  --save-dir $CheckpointDir
if ($LASTEXITCODE -ne 0) { throw "PPO training failed with exit code $LASTEXITCODE" }

$checkpoint = (Resolve-Path (Join-Path $CheckpointDir "ppo_model_final.npz")).Path
$env:MODEL_PATH = $checkpoint

Write-Host "[2/3] Standalone PPO evaluation"
$env:SMART_FACTORY_SIM_DURATION = [string]$Duration
python scripts/run_experiments.py `
  --standalone `
  --scenario A B C `
  --scheduler PPO_RL `
  --seeds $Seeds
if ($LASTEXITCODE -ne 0) { throw "Standalone PPO evaluation failed with exit code $LASTEXITCODE" }

if ($RunWebots) {
    if (-not (Test-Path $Webots)) { throw "Webots executable not found: $Webots" }
    Write-Host "[3/3] Headless Webots PPO evaluation"
    # run_experiments.py launches Webots with --batch, --no-rendering and
    # --mode=fast. SMART_FACTORY_SIM_DURATION controls automatic termination.
    python scripts/run_experiments.py `
      --scenario A B C `
      --scheduler PPO_RL `
      --seeds $Seeds `
      --webots $Webots
    if ($LASTEXITCODE -ne 0) { throw "Webots PPO evaluation failed with exit code $LASTEXITCODE" }
} else {
    Write-Host "[3/3] Webots evaluation skipped. Add -RunWebots to enable it."
}

Remove-Item Env:MODEL_PATH -ErrorAction SilentlyContinue
Write-Host "PPO checkpoint: $checkpoint"
Write-Host "Metrics: results\comparison_report.txt and results\experiment_*_PPO_RL_*.json"
