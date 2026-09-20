$ErrorActionPreference = "Stop"
$RepoRoot = Split-Path -Parent $PSScriptRoot
$AgentRoot = Join-Path $RepoRoot "agents/python-reference-brain"

Push-Location $AgentRoot
try {
    python -m pip install --disable-pip-version-check -e .
    python -m unittest discover -s tests -v
}
finally {
    Pop-Location
}

