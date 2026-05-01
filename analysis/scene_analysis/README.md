# Scene Analysis

This directory holds the retained-scene analysis scripts for the current
`main` branch.

The scripts use the supported-scene manifest in
[metadata/supported_nuscenes_scenes.txt](../../metadata/supported_nuscenes_scenes.txt),
not the full `979`-scene CAN-bus inventory.

Local data paths are read from `analysis/paths.json`, which is intentionally
ignored. Start from `analysis/paths.example.json`.

## Layout

- [all_scenes](all_scenes/)
  runs the supported scene set with the default GPS seed and keeps a retained
  summary.
- [startup_diagnostics](startup_diagnostics/)
  reruns the supported scene set and writes the retained startup-trace,
  startup-readiness, and startup-handoff PNGs for every scene.
- [common.py](common.py)
  holds the shared helpers used by these scripts.

## Retained results

Each script writes under its own folder in `analysis/scene_analysis`. Nothing
from this workflow is written to the repo-root `output/` directory.

The all-scenes sweep writes retained summaries under
`all_scenes/results/<run_label>/` and uses `all_scenes/scratch/<run_label>/`
only while it is running.

## How the scripts fit together

1. `all_scenes/run_all_scenes.py` runs the supported scene set and writes the
   retained summary for the current branch and commit.
2. `startup_diagnostics/run_startup_diagnostics.py` reruns the supported scene
   set and writes the retained startup plots for every scene.

## How to run it

```bash
cp analysis/paths.example.json analysis/paths.json
edit analysis/paths.json
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build --parallel "$(nproc)"
ctest --test-dir build --output-on-failure
python analysis/scene_analysis/all_scenes/run_all_scenes.py
python analysis/scene_analysis/startup_diagnostics/run_startup_diagnostics.py
```

The scripts have no CLI. The current paths and settings live in constants at
the top of each file.
