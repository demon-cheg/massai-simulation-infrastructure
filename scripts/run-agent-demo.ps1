$ErrorActionPreference = "Stop"
$RepoRoot = Split-Path -Parent $PSScriptRoot
$AgentRoot = Join-Path $RepoRoot "agents/python-reference-brain"
$Scenario = Join-Path $RepoRoot "scenarios/reference-demo.json"

Push-Location $AgentRoot
try {
    python -m pip install --disable-pip-version-check -e .
    python -m networked_simulation_agent --scenario $Scenario
}
finally {
    Pop-Location
}
