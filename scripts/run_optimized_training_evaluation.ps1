param(
    [ValidateSet("Quick", "Full")]
    [string]$Profile = "Full",
    [string]$Webots = "C:\Program Files\Webots\msys64\mingw64\bin\webots.exe",
    [switch]$StandaloneOnly,
    [switch]$SkipTraining
)

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
$ModelRoot = Join-Path $Root "results\hyperparameter_search\optimized_v2"
$ReportRoot = Join-Path $Root "results\optimized_v2"
New-Item -ItemType Directory -Force -Path $ModelRoot, $ReportRoot | Out-Null

if (-not $StandaloneOnly -and -not (Test-Path -LiteralPath $Webots -PathType Leaf)) {
    throw "Webots executable not found: $Webots"
}

if ($Profile -eq "Quick") {
    $SarsaEpisodes = 300
    $DqnEpisodes = 300
    $DqnWarmup = 256
    $PpoEpisodes = 50
    $EvalSeeds = 3
    $Duration = 120
} else {
    $SarsaEpisodes = 10000
    $DqnEpisodes = 5000
    $DqnWarmup = 5000
    $PpoEpisodes = 5000
    $EvalSeeds = 1
    $Duration = 1800
}

$env:PYTHONDONTWRITEBYTECODE = "1"
$env:SMART_FACTORY_SIM_DURATION = [string]$Duration
$env:SA_INITIAL_TEMPERATURE = "10.0"
$env:SA_COOLING_RATE = "0.985"
$env:SA_MIN_TEMPERATURE = "0.01"
$env:SA_MAX_ITERATIONS = "2000"
$env:SA_TIME_BUDGET_MS = "5"
$env:GA_POPULATION = "48"
$env:GA_GENERATIONS = "50"
$env:GA_TIME_BUDGET_MS = "10"

$SarsaDir = Join-Path $ModelRoot "best\sarsa"
$DqnDir = Join-Path $ModelRoot "best\dqn"
$PpoDir = Join-Path $ModelRoot "best\ppo"

if (-not $SkipTraining) {
    python (Join-Path $PSScriptRoot "search_hyperparameters.py") `
        --profile $($Profile.ToLowerInvariant()) --output $ModelRoot
}

$SearchReport = Get-Content (Join-Path $ModelRoot "search_report.json") -Raw | ConvertFrom-Json
$BestSa = $SearchReport.algorithms.sa.best.config
$BestGa = $SearchReport.algorithms.ga.best.config
$env:SA_INITIAL_TEMPERATURE = [string]$BestSa.initial_temperature
$env:SA_COOLING_RATE = [string]$BestSa.cooling_rate
$env:SA_MIN_TEMPERATURE = [string]$BestSa.minimum_temperature
$env:SA_MAX_ITERATIONS = [string]$BestSa.max_iterations
$env:SA_TIME_BUDGET_MS = [string]$BestSa.time_budget_ms
$env:GA_POPULATION = [string]$BestGa.population_size
$env:GA_GENERATIONS = [string]$BestGa.max_generations
$env:GA_TIME_BUDGET_MS = [string]$BestGa.time_budget_ms

$RequiredModels = @(
    (Join-Path $SarsaDir "best_validation.json"),
    (Join-Path $DqnDir "best_validation.pkl"),
    (Join-Path $PpoDir "ppo_model_best_validation.npz")
)
foreach ($Model in $RequiredModels) {
    if (-not (Test-Path -LiteralPath $Model -PathType Leaf)) {
        throw "Promoted model not found: $Model"
    }
}

python (Join-Path $PSScriptRoot "evaluate_scheduler.py") `
    --algorithm sarsa --checkpoint (Join-Path $SarsaDir "best_validation.json") `
    --split test --seeds 1 | Tee-Object (Join-Path $ReportRoot "sarsa_abstract_test.json")
python (Join-Path $PSScriptRoot "evaluate_scheduler.py") `
    --algorithm dqn --checkpoint (Join-Path $DqnDir "best_validation.pkl") `
    --split test --seeds 1 | Tee-Object (Join-Path $ReportRoot "dqn_abstract_test.json")

$Runner = Join-Path $PSScriptRoot "run_experiments.py"
# Evaluate every non-model scheduler together. SA and GA use the promoted
# hyperparameters loaded from search_report.json above; the other entries are
# deterministic/seeded baselines and do not require a checkpoint.
$RunArgs = @(
    "--scenario", "A", "B", "C",
    "--scheduler",
    "FCFS", "NearestNeighbour", "RoundRobin", "Greedy", "Random",
    "Hungarian", "Auction", "SA", "GA", "SARSA", "DQN", "PPO_RL",
    "--seeds", $EvalSeeds,
    "--sarsa-model", (Join-Path $SarsaDir "best_validation.json"),
    "--dqn-model", (Join-Path $DqnDir "best_validation.pkl"),
    "--ppo-model", (Join-Path $PpoDir "ppo_model_best_validation.npz")
)
if ($StandaloneOnly) { $RunArgs += "--standalone" } else { $RunArgs += @("--webots", $Webots) }
python $Runner @RunArgs

$ComparisonReport = Join-Path $Root "results\comparison_report.txt"
if (-not (Test-Path -LiteralPath $ComparisonReport -PathType Leaf)) {
    throw "Unified comparison report was not generated: $ComparisonReport"
}
Copy-Item -LiteralPath $ComparisonReport `
    -Destination (Join-Path $ReportRoot "all_algorithms_webots_comparison.txt") `
    -Force
Copy-Item -LiteralPath (Join-Path $ModelRoot "search_report.json") `
    -Destination (Join-Path $ReportRoot "hyperparameter_search_report.json") `
    -Force

Write-Host "Completed. Models: $ModelRoot"
Write-Host "Reports/results: $ReportRoot and $(Join-Path $Root 'results')"
