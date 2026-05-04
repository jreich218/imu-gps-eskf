# Startup Diagnostics

This directory holds the retained startup-plot workflow for the supported
scene set on the current `main` branch.

- [run_startup_diagnostics.py](run_startup_diagnostics.py)
  reruns every supported scene and writes three retained startup plot families:
  trace overview, readiness overview, and handoff overview.
- Each run uses all CPU cores.
- Raw per-scene app runs and startup traces are written under `scratch/` while
  the script is running and are deleted after the retained PNGs and summaries
  are written.

Retained output lives under `results/<run_label>/`.
