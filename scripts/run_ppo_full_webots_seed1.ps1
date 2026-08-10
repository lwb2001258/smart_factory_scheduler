<#
Run the complete PPO workflow with the dissertation evaluation settings:
  - Full PPO hyperparameter optimisation
  - Route-aware PPO deployment
  - Webots evaluation on Scenes A, B and C
  - Explicit random seed value 1
  - 1800 simulation seconds per scene

Run from the project root:
  .\scripts\run_ppo_full_webots_seed1.ps1
#>
[CmdletBinding()]
param()

$ErrorActionPreference = "Stop"
$ProjectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$PipelineScript = Join-Path $PSScriptRoot "run_ppo_auto_optimize_webots.ps1"
$WebotsExecutable = "C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"

if (-not (Test-Path -LiteralPath $PipelineScript -PathType Leaf)) {
    throw "PPO pipeline script not found: $PipelineScript"
}
if (-not (Test-Path -LiteralPath $WebotsExecutable -PathType Leaf)) {
    throw "Webots executable not found: $WebotsExecutable"
}

Set-Location $ProjectRoot

Write-Host "Starting full PPO optimisation and Webots A/B/C evaluation"
Write-Host "Random seed: 1"
Write-Host "Duration: 1800 simulation seconds per scene"
Write-Host "This is the full training profile and may take a long time."

& $PipelineScript `
    -Profile Full `
    -EvaluationSeedValues 1 `
    -Duration 1800 `
    -RouteWeight 0.50 `
    -TaskCandidates 8 `
    -Webots $WebotsExecutable `
    -OutputDirectory "results\ppo_auto_optimized_full_seed1"

if ($LASTEXITCODE -ne 0) {
    throw "Full PPO workflow failed with exit code $LASTEXITCODE"
}

Write-Host "Full PPO workflow completed successfully."
Write-Host "Models: results\ppo_auto_optimized_full_seed1"
Write-Host "Metrics: results\experiment_[A-C]_PPO_RL_*.json"
