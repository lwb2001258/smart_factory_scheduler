param(
    [Parameter(Mandatory=$true)][string]$BaseCheckpoint,
    [ValidateSet("DQN", "SARSA", "PPO", "SARSA_LAMBDA", "RAINBOW_DQN", "A2C", "DISCRETE_SAC", "QR_DQN")][string]$Algorithm = "DQN",
    [string]$Webots = "C:\Program Files\Webots\msys64\mingw64\bin\webots.exe",
    [ValidateSet("A", "B", "C")][string]$Scenario = "C",
    [string[]]$CollectionSeeds = @("300001", "300002"),
    [string[]]$ValidationSeeds = @("310001", "310002"),
    [double]$Duration = 120,
    [string]$OutputDir = "",
    [int]$Updates = 100,
    [switch]$Resume,
    [ValidateRange(1, 5)][int]$WebotsRetries = 2,
    [ValidateRange(0.01, 5.0)][double]$RlTimeoutSeconds = 0.5
)
$ErrorActionPreference = "Stop"

function ConvertTo-SeedList {
    param([string[]]$Values, [string]$Name)
    $Parsed = @()
    foreach ($Value in $Values) {
        foreach ($Token in ($Value -split '[,;\s]+')) {
            if ([string]::IsNullOrWhiteSpace($Token)) { continue }
            $Number = 0
            if (-not [int]::TryParse($Token, [ref]$Number)) {
                throw "Invalid seed in ${Name}: '$Token'"
            }
            $Parsed += $Number
        }
    }
    if ($Parsed.Count -eq 0) { throw "$Name must contain at least one seed" }
    if (($Parsed | Select-Object -Unique).Count -ne $Parsed.Count) {
        throw "$Name contains duplicate seeds"
    }
    return [int[]]$Parsed
}

$CollectionSeedValues = ConvertTo-SeedList $CollectionSeeds "CollectionSeeds"
$ValidationSeedValues = ConvertTo-SeedList $ValidationSeeds "ValidationSeeds"
$Overlap = @($CollectionSeedValues | Where-Object { $ValidationSeedValues -contains $_ })
if ($Overlap.Count -gt 0) {
    throw "CollectionSeeds and ValidationSeeds overlap: $($Overlap -join ',')"
}
if (-not (Test-Path -LiteralPath $Webots -PathType Leaf)) { throw "Webots not found: $Webots" }
if (-not (Test-Path -LiteralPath $BaseCheckpoint -PathType Leaf)) { throw "Checkpoint not found: $BaseCheckpoint" }
$Root = Split-Path -Parent $PSScriptRoot
if ([string]::IsNullOrWhiteSpace($OutputDir)) {
    $OutputDir = "results\webots_finetune\$($Algorithm.ToLower())_v2"
}
$ResolvedOutput = Join-Path $Root $OutputDir
New-Item -ItemType Directory -Force -Path $ResolvedOutput | Out-Null
$env:PYTHONDONTWRITEBYTECODE = "1"
$PythonArgs = @(
    (Join-Path $PSScriptRoot "run_webots_finetune_workflow.py"),
    "--base-checkpoint", (Resolve-Path $BaseCheckpoint).Path,
    "--algorithm", $Algorithm,
    "--webots", $Webots, "--scenario", $Scenario,
    "--collection-seeds"
) + $CollectionSeedValues + @(
    "--validation-seeds"
) + $ValidationSeedValues + @(
    "--duration", $Duration, "--output-dir", $ResolvedOutput,
    "--updates", $Updates, "--webots-retries", $WebotsRetries,
    "--rl-timeout-seconds", $RlTimeoutSeconds
)
if ($Resume) { $PythonArgs += "--resume" }
$PreviousErrorPreference = $ErrorActionPreference
$ErrorActionPreference = "Continue"
& python @PythonArgs 2>&1 | Tee-Object -FilePath (Join-Path $ResolvedOutput "workflow_console.log")
$PythonExitCode = $LASTEXITCODE
$ErrorActionPreference = $PreviousErrorPreference
if ($PythonExitCode -ne 0) {
    throw "Webots fine-tuning workflow failed; inspect $ResolvedOutput\workflow_error.json and workflow_console.log"
}
