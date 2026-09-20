# Python reference Brain

This package is deliberately small. It proves that autonomy is an external, replaceable workload and gives the simulation a deterministic integration oracle before a real model exists.

The controller uses proportional guidance and a finite-state mission executor. `TargetObservation` can initially come from Unity's ground-truth perception adapter. Replacing it with model output must not require changes to mission logic.

Run from this directory:

```bash
python -m pip install -e .
python -m networked_simulation_agent --scenario ../../scenarios/reference-demo.json
python -m unittest discover -s tests -v
```
