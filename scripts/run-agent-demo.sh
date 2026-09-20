#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
agent_root="$repo_root/agents/python-reference-brain"

python3 -m pip install --disable-pip-version-check -e "$agent_root"
python3 -m networked_simulation_agent --scenario "$repo_root/scenarios/reference-demo.json"
