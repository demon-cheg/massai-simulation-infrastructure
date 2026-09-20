from __future__ import annotations

import contextlib
import io
from pathlib import Path
import unittest

from networked_simulation_agent.harness import run_demo


class DemoTests(unittest.TestCase):
    def test_reference_scenario_completes(self) -> None:
        scenario = Path(__file__).resolve().parents[3] / "scenarios" / "reference-demo.json"
        with contextlib.redirect_stdout(io.StringIO()):
            result = run_demo(scenario)
        self.assertEqual(result["commands_succeeded"], 7)
        self.assertLess(result["final_position_enu_m"][2], 0.05)


if __name__ == "__main__":
    unittest.main()
