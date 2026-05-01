#!/usr/bin/env python3

from __future__ import annotations

import concurrent.futures
import os
import shutil
import statistics
import sys
import time
from pathlib import Path


SCENE_ANALYSIS_DIR = Path(__file__).resolve().parents[1]
if str(SCENE_ANALYSIS_DIR) not in sys.path:
    sys.path.insert(0, str(SCENE_ANALYSIS_DIR))

from common import (  # noqa: E402
    current_branch_name,
    current_commit_short_sha,
    current_run_label,
    ensure_binaries_exist,
    list_scene_ids,
    make_clean_dir,
    run_app_for_scene,
    write_json,
)


RUN_LABEL = current_run_label()
WORKER_COUNT = max(1, int(os.cpu_count() or 1))
RESULTS_ROOT = Path(__file__).resolve().parent / "results" / RUN_LABEL
SCRATCH_ROOT = Path(__file__).resolve().parent / "scratch" / RUN_LABEL
RUNS_ROOT = SCRATCH_ROOT / "app_runs"
DEFAULT_GPS_SEED = 1234


def retained_non_beating_scene_entry(result: dict) -> dict:
    return {
        "scene_id": str(result["scene_id"]),
        "raw_gps_rmse_xy_m": float(result["raw_gps_rmse_xy"]),
        "eskf_rmse_xy_m": float(result["eskf_rmse_xy"]),
    }


def summary_markdown(summary: dict) -> str:
    lines = [
        f"- GPS seed: `{DEFAULT_GPS_SEED}`",
        f"- completed: `{summary['completed_count']}`",
        f"- failed: `{summary['failed_count']}`",
        f"- median raw GPS RMSE: `{summary['raw_gps_rmse_xy_median_m']:.5f} m`",
        f"- mean raw GPS RMSE: `{summary['raw_gps_rmse_xy_mean_m']:.16f} m`",
        f"- median ESKF RMSE: `{summary['eskf_rmse_xy_median_m']:.6f} m`",
        f"- mean ESKF RMSE: `{summary['eskf_rmse_xy_mean_m']:.16f} m`",
        (
            f"- beats raw GPS: `{summary['beats_raw_count']} / "
            f"{summary['completed_count']}`"
        ),
        f"- `>= 2x` worse than raw GPS: `{summary['two_x_worse_count']}`",
        (
            "- failed scenes: "
            + ", ".join(summary["failed_scenes"])
            if summary["failed_scenes"]
            else "- failed scenes: none"
        ),
    ]

    if summary["completed_non_beating_scenes"]:
        scene_text = ", ".join(
            (
                f"{entry['scene_id']} with raw {entry['raw_gps_rmse_xy_m']:.5f} m "
                f"and ESKF {entry['eskf_rmse_xy_m']:.5f} m"
            )
            for entry in summary["completed_non_beating_scenes"]
        )
        lines.append(f"- completed non-beating scenes: {scene_text}")
    else:
        lines.append("- completed non-beating scenes: none")

    lines.append(f"- total sweep time: `{summary['total_elapsed_s']:.2f} s`")
    return "\n".join(lines) + "\n"


def main() -> None:
    ensure_binaries_exist(require_trace=False)

    scene_ids = list_scene_ids()
    make_clean_dir(RESULTS_ROOT)
    make_clean_dir(SCRATCH_ROOT)

    start_time = time.perf_counter()
    with concurrent.futures.ThreadPoolExecutor(max_workers=WORKER_COUNT) as executor:
        futures = {
            executor.submit(run_app_for_scene, scene_id, RUNS_ROOT): scene_id
            for scene_id in scene_ids
        }

        results = []
        for future in concurrent.futures.as_completed(futures):
            results.append(future.result())

    total_elapsed_s = time.perf_counter() - start_time
    results.sort(key=lambda result: str(result["scene_id"]))

    completed_results = [
        result for result in results if str(result["status"]) == "completed"
    ]
    failed_results = [result for result in results if str(result["status"]) == "failed"]

    raw_rmse_values = [
        float(result["raw_gps_rmse_xy"]) for result in completed_results
    ]
    eskf_rmse_values = [
        float(result["eskf_rmse_xy"]) for result in completed_results
    ]
    beats_raw_results = [
        result
        for result in completed_results
        if float(result["eskf_rmse_xy"]) < float(result["raw_gps_rmse_xy"])
    ]
    non_beating_results = [
        result
        for result in completed_results
        if float(result["eskf_rmse_xy"]) >= float(result["raw_gps_rmse_xy"])
    ]
    two_x_worse_results = [
        result
        for result in completed_results
        if float(result["eskf_rmse_xy"]) >= 2.0 * float(result["raw_gps_rmse_xy"])
    ]

    summary = {
        "branch": current_branch_name(),
        "commit": current_commit_short_sha(),
        "gps_seed": DEFAULT_GPS_SEED,
        "completed_count": len(completed_results),
        "completed_non_beating_scenes": [
            retained_non_beating_scene_entry(result) for result in non_beating_results
        ],
        "failed_count": len(failed_results),
        "failed_scenes": [str(result["scene_id"]) for result in failed_results],
        "raw_gps_rmse_xy_mean_m": statistics.fmean(raw_rmse_values),
        "raw_gps_rmse_xy_median_m": statistics.median(raw_rmse_values),
        "run_label": RUN_LABEL,
        "scene_count": len(scene_ids),
        "total_elapsed_s": total_elapsed_s,
        "two_x_worse_count": len(two_x_worse_results),
        "worker_count": WORKER_COUNT,
        "beats_raw_count": len(beats_raw_results),
        "eskf_rmse_xy_mean_m": statistics.fmean(eskf_rmse_values),
        "eskf_rmse_xy_median_m": statistics.median(eskf_rmse_values),
    }

    summary_json_path = RESULTS_ROOT / "summary.json"
    summary_md_path = RESULTS_ROOT / "summary.md"
    write_json(summary_json_path, summary)
    summary_md_path.write_text(summary_markdown(summary))

    shutil.rmtree(SCRATCH_ROOT)
    try:
        SCRATCH_ROOT.parent.rmdir()
    except OSError:
        pass

    print(f"Wrote {summary_json_path}")
    print(f"Wrote {summary_md_path}")


if __name__ == "__main__":
    main()
