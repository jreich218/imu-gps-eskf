#!/home/dfw/dev/python_laboratory/.venv/bin/python

from __future__ import annotations

import csv
import json
import math
import os
import shutil
import subprocess
import tempfile
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from statistics import mean, median

REPO_ROOT = Path(__file__).resolve().parents[1]
APP_PATH = REPO_ROOT / "build" / "imu-gps-eskf"
SCENE_ROOT = Path("/data/sets/nuscenes/can_bus")
SUPPORTED_SCENES_PATH = REPO_ROOT / "metadata" / "supported_nuscenes_scenes.txt"
SEED_START = 1
SEED_END = 100
WORKER_COUNT = max(1, os.cpu_count() or 1)
RESULTS_ROOT = REPO_ROOT / "analysis" / "supported_scene_seed_sweep"


@dataclass
class SceneRunResult:
    seed: int
    scene_id: str
    succeeded: bool
    raw_gps_rmse_xy: float | None
    eskf_rmse_xy: float | None
    stderr: str


def git_commit_sha() -> str:
    return subprocess.check_output(
        ["git", "rev-parse", "--short", "HEAD"],
        cwd=REPO_ROOT,
        text=True,
    ).strip()


def read_supported_scenes() -> list[str]:
    with SUPPORTED_SCENES_PATH.open("r", encoding="utf-8") as handle:
        return [line.strip() for line in handle if line.strip()]


def ensure_app_exists() -> None:
    if not APP_PATH.is_file():
        raise RuntimeError(f"Missing app binary: {APP_PATH}")


def parse_summary_value(stdout_text: str, prefix: str) -> float:
    for line in stdout_text.splitlines():
        if line.startswith(prefix):
            value_text = line[len(prefix) :].strip()
            if value_text.endswith(" m"):
                value_text = value_text[:-2]
            return float(value_text)
    raise RuntimeError(f"Could not find summary line: {prefix!r}")


def link_scene_files(scenarios_dir: Path, scene_id: str) -> None:
    for suffix in ("_pose.json", "_ms_imu.json", "_zoe_veh_info.json"):
        source = SCENE_ROOT / f"{scene_id}{suffix}"
        target = scenarios_dir / source.name
        if not source.is_file():
            raise RuntimeError(f"Missing scene file: {source}")
        target.symlink_to(source)


def run_one_scene(seed: int, scene_id: str) -> SceneRunResult:
    with tempfile.TemporaryDirectory(prefix=f"imu_gps_eskf_{scene_id}_seed_{seed:03d}_") as temp_dir_text:
        temp_dir = Path(temp_dir_text)
        scenarios_dir = temp_dir / "scenarios"
        scenarios_dir.mkdir(parents=True, exist_ok=True)
        link_scene_files(scenarios_dir, scene_id)

        env = os.environ.copy()
        env["IMU_GPS_ESKF_GPS_SEED"] = str(seed)

        completed = subprocess.run(
            [str(APP_PATH)],
            cwd=temp_dir,
            env=env,
            text=True,
            capture_output=True,
        )

        if completed.returncode != 0:
            return SceneRunResult(
                seed=seed,
                scene_id=scene_id,
                succeeded=False,
                raw_gps_rmse_xy=None,
                eskf_rmse_xy=None,
                stderr=completed.stderr.strip(),
            )

        raw_gps_rmse_xy = parse_summary_value(completed.stdout, "RMSE raw GPS (xy):")
        eskf_rmse_xy = parse_summary_value(completed.stdout, "RMSE ESKF (xy):")
        return SceneRunResult(
            seed=seed,
            scene_id=scene_id,
            succeeded=True,
            raw_gps_rmse_xy=raw_gps_rmse_xy,
            eskf_rmse_xy=eskf_rmse_xy,
            stderr="",
        )


def percentile_range(values: list[float]) -> tuple[float, float, float, float]:
    return min(values), median(values), mean(values), max(values)


def write_csv(path: Path, fieldnames: list[str], rows: list[dict[str, object]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def summary_markdown(summary: dict[str, object]) -> str:
    lines = [
        f"# Supported scene seed sweep ({summary['commit_sha']})",
        "",
        f"- generated_at_utc: `{summary['generated_at_utc']}`",
        f"- supported_scene_count: `{summary['supported_scene_count']}`",
        f"- seed_range: `{summary['seed_start']}..{summary['seed_end']}`",
        f"- worker_count: `{summary['worker_count']}`",
        "",
        "## Counts",
        "",
        f"- completed count range: `{summary['completed_count_min']} .. {summary['completed_count_max']}`",
        f"- failed count range: `{summary['failed_count_min']} .. {summary['failed_count_max']}`",
        f"- beats-raw count range: `{summary['beats_raw_count_min']} .. {summary['beats_raw_count_max']}`",
        f"- non-beating count range: `{summary['non_beating_count_min']} .. {summary['non_beating_count_max']}`",
        f"- mean non-beating count: `{summary['mean_non_beating_count']}`",
        f"- total non-beating scene-seed cases: `{summary['total_non_beating_cases']}`",
        "",
        "## RMSE ranges",
        "",
        f"- median raw GPS RMSE range: `{summary['median_raw_gps_rmse_min']} .. {summary['median_raw_gps_rmse_max']} m`",
        f"- mean raw GPS RMSE range: `{summary['mean_raw_gps_rmse_min']} .. {summary['mean_raw_gps_rmse_max']} m`",
        f"- median ESKF RMSE range: `{summary['median_eskf_rmse_min']} .. {summary['median_eskf_rmse_max']} m`",
        f"- mean ESKF RMSE range: `{summary['mean_eskf_rmse_min']} .. {summary['mean_eskf_rmse_max']} m`",
        "",
        "## Artifacts",
        "",
        "- `per_seed_summary.csv`",
        "- `non_beating_cases.csv`",
        "- `failed_cases.csv`",
        "- `summary.json`",
    ]
    return "\n".join(lines) + "\n"


def main() -> None:
    ensure_app_exists()
    supported_scenes = read_supported_scenes()
    commit_sha = git_commit_sha()

    results_dir = RESULTS_ROOT / commit_sha
    if results_dir.exists():
        shutil.rmtree(results_dir)
    results_dir.mkdir(parents=True, exist_ok=True)

    all_results: list[SceneRunResult] = []
    for seed in range(SEED_START, SEED_END + 1):
        print(
            f"Running supported-scene sweep for seed {seed} with {WORKER_COUNT} workers",
            flush=True,
        )
        with ThreadPoolExecutor(max_workers=WORKER_COUNT) as executor:
            seed_results = list(executor.map(lambda scene_id: run_one_scene(seed, scene_id), supported_scenes))
        all_results.extend(seed_results)

    per_seed_rows: list[dict[str, object]] = []
    non_beating_rows: list[dict[str, object]] = []
    failed_rows: list[dict[str, object]] = []

    completed_counts: list[int] = []
    failed_counts: list[int] = []
    beats_raw_counts: list[int] = []
    non_beating_counts: list[int] = []
    median_raw_values: list[float] = []
    mean_raw_values: list[float] = []
    median_eskf_values: list[float] = []
    mean_eskf_values: list[float] = []

    for seed in range(SEED_START, SEED_END + 1):
        seed_results = [result for result in all_results if result.seed == seed]
        completed = [result for result in seed_results if result.succeeded]
        failed = [result for result in seed_results if not result.succeeded]
        non_beating = [
            result
            for result in completed
            if result.eskf_rmse_xy is not None
            and result.raw_gps_rmse_xy is not None
            and result.eskf_rmse_xy >= result.raw_gps_rmse_xy
        ]
        beats_raw = len(completed) - len(non_beating)

        raw_values = [result.raw_gps_rmse_xy for result in completed if result.raw_gps_rmse_xy is not None]
        eskf_values = [result.eskf_rmse_xy for result in completed if result.eskf_rmse_xy is not None]

        completed_counts.append(len(completed))
        failed_counts.append(len(failed))
        beats_raw_counts.append(beats_raw)
        non_beating_counts.append(len(non_beating))
        median_raw_values.append(median(raw_values))
        mean_raw_values.append(mean(raw_values))
        median_eskf_values.append(median(eskf_values))
        mean_eskf_values.append(mean(eskf_values))

        per_seed_rows.append(
            {
                "seed": seed,
                "completed_count": len(completed),
                "failed_count": len(failed),
                "beats_raw_count": beats_raw,
                "non_beating_count": len(non_beating),
                "median_raw_gps_rmse_xy_m": median(raw_values),
                "mean_raw_gps_rmse_xy_m": mean(raw_values),
                "median_eskf_rmse_xy_m": median(eskf_values),
                "mean_eskf_rmse_xy_m": mean(eskf_values),
            }
        )

        for result in non_beating:
            non_beating_rows.append(
                {
                    "seed": result.seed,
                    "scene_id": result.scene_id,
                    "raw_gps_rmse_xy_m": result.raw_gps_rmse_xy,
                    "eskf_rmse_xy_m": result.eskf_rmse_xy,
                    "ratio": result.eskf_rmse_xy / result.raw_gps_rmse_xy,
                }
            )

        for result in failed:
            failed_rows.append(
                {
                    "seed": result.seed,
                    "scene_id": result.scene_id,
                    "stderr": result.stderr,
                }
            )

    summary = {
        "commit_sha": commit_sha,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "supported_scene_count": len(supported_scenes),
        "seed_start": SEED_START,
        "seed_end": SEED_END,
        "worker_count": WORKER_COUNT,
        "completed_count_min": min(completed_counts),
        "completed_count_max": max(completed_counts),
        "failed_count_min": min(failed_counts),
        "failed_count_max": max(failed_counts),
        "beats_raw_count_min": min(beats_raw_counts),
        "beats_raw_count_max": max(beats_raw_counts),
        "non_beating_count_min": min(non_beating_counts),
        "non_beating_count_max": max(non_beating_counts),
        "mean_non_beating_count": mean(non_beating_counts),
        "total_non_beating_cases": len(non_beating_rows),
        "median_raw_gps_rmse_min": min(median_raw_values),
        "median_raw_gps_rmse_max": max(median_raw_values),
        "mean_raw_gps_rmse_min": min(mean_raw_values),
        "mean_raw_gps_rmse_max": max(mean_raw_values),
        "median_eskf_rmse_min": min(median_eskf_values),
        "median_eskf_rmse_max": max(median_eskf_values),
        "mean_eskf_rmse_min": min(mean_eskf_values),
        "mean_eskf_rmse_max": max(mean_eskf_values),
    }

    write_csv(
        results_dir / "per_seed_summary.csv",
        [
            "seed",
            "completed_count",
            "failed_count",
            "beats_raw_count",
            "non_beating_count",
            "median_raw_gps_rmse_xy_m",
            "mean_raw_gps_rmse_xy_m",
            "median_eskf_rmse_xy_m",
            "mean_eskf_rmse_xy_m",
        ],
        per_seed_rows,
    )
    write_csv(
        results_dir / "non_beating_cases.csv",
        ["seed", "scene_id", "raw_gps_rmse_xy_m", "eskf_rmse_xy_m", "ratio"],
        non_beating_rows,
    )
    write_csv(
        results_dir / "failed_cases.csv",
        ["seed", "scene_id", "stderr"],
        failed_rows,
    )

    with (results_dir / "summary.json").open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)
        handle.write("\n")

    with (results_dir / "summary.md").open("w", encoding="utf-8") as handle:
        handle.write(summary_markdown(summary))


if __name__ == "__main__":
    main()
