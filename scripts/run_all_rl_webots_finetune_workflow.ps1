param(
    [Parameter(Mandatory=$true)][string]$DqnCheckpoint,
    [Parameter(Mandatory=$true)][string]$SarsaCheckpoint,
    [Parameter(Mandatory=$true)][string]$PpoCheckpoint,
    [Parameter(Mandatory=$true)][string]$SarsaLambdaCheckpoint,
    [Parameter(Mandatory=$true)][string]$RainbowDqnCheckpoint,
    [Parameter(Mandatory=$true)][string]$A2cCheckpoint,
    [Parameter(Mandatory=$true)][string]$DiscreteSacCheckpoint,
    [Parameter(Mandatory=$true)][string]$QrDqnCheckpoint,
    [string]$Webots = "C:\Program Files\Webots\msys64\mingw64\bin\webots.exe",
    [ValidateSet("A", "B", "C")][string]$Scenario = "C",
    [string[]]$CollectionSeeds = @("300001", "300002"),
    [string[]]$ValidationSeeds = @("310001", "310002"),
    [double]$Duration = 120,
    [string]$OutputRoot = "results\webots_finetune\all_rl_v2",
    [int]$Updates = 100,
    [switch]$Resume
)
$ErrorActionPreference = "Stop"
$Workflow = Join-Path $PSScriptRoot "run_webots_finetune_workflow.ps1"
$Models = [ordered]@{
    DQN = $DqnCheckpoint; SARSA = $SarsaCheckpoint; PPO = $PpoCheckpoint
    SARSA_LAMBDA = $SarsaLambdaCheckpoint
    RAINBOW_DQN = $RainbowDqnCheckpoint; A2C = $A2cCheckpoint
    DISCRETE_SAC = $DiscreteSacCheckpoint; QR_DQN = $QrDqnCheckpoint
}
foreach ($Entry in $Models.GetEnumerator()) {
    $Target = Join-Path $OutputRoot $Entry.Key.ToLower()
    $Arguments = @{
        BaseCheckpoint = $Entry.Value; Algorithm = $Entry.Key
        Webots = $Webots; Scenario = $Scenario
        CollectionSeeds = $CollectionSeeds; ValidationSeeds = $ValidationSeeds
        Duration = $Duration; OutputDir = $Target; Updates = $Updates
    }
    if ($Resume) { $Arguments.Resume = $true }
    & $Workflow @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "All-RL workflow stopped at $($Entry.Key)"
    }
}
