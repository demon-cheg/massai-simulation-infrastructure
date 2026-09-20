from __future__ import annotations

import argparse
from pathlib import Path

from .harness import run_demo


def _default_scenario() -> Path:
    return Path(__file__).resolve().parents[4] / "scenarios" / "reference-demo.json"


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run the MassAI Networked Simulation Infrastructure reference Brain harness"
    )
    parser.add_argument("--scenario", type=Path, default=_default_scenario())
    parser.add_argument("--max-steps", type=int, default=5000)
    args = parser.parse_args()
    run_demo(args.scenario.resolve(), args.max_steps)


if __name__ == "__main__":
    main()
