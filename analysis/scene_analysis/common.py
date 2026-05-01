from __future__ import annotations

import csv
import json
import os
import shutil
import subprocess
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
BUILD_DIR = REPO_ROOT / "build"
APP_BIN = BUILD_DIR / "imu-gps-eskf"
TRACE_BIN = BUILD_DIR / "dump_startup_trace"
CAN_BUS_DIR = Path(
    "/data/sets/nuscenes/can_bus"
)  # Replace with path to user-downloaded nuScenes CAN Bus files


def load_json(path: Path) -> object:
    return json.loads(path.read_text())


def write_json(path: Path, obj: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(obj, indent=2, sort_keys=True) + "\n")


def parse_run_summary(stdout_text: str) -> dict[str, float | int | str]:
    summary: dict[str, float | int | str] = {}
    for line in stdout_text.splitlines():
        line = line.strip()
        if line.startswith("GPS updates: "):
            summary["gps_updates"] = int(line.removeprefix("GPS updates: "))
        elif line.startswith("RMSE raw GPS (xy): "):
            summary["raw_gps_rmse_xy"] = float(
                line.removeprefix("RMSE raw GPS (xy): ").removesuffix(" m")
            )
        elif line.startswith("RMSE ESKF (xy): "):
            summary["eskf_rmse_xy"] = float(
                line.removeprefix("RMSE ESKF (xy): ").removesuffix(" m")
            )
        elif line.startswith("Wrote log: "):
            summary["log_path"] = line.removeprefix("Wrote log: ")

    required_keys = {
        "gps_updates",
        "raw_gps_rmse_xy",
        "eskf_rmse_xy",
        "log_path",
    }
    missing_keys = required_keys.difference(summary)
    if missing_keys:
        missing_text = ", ".join(sorted(missing_keys))
        raise RuntimeError(f"Could not parse run summary. Missing: {missing_text}")

    return summary


def ensure_binaries_exist(*, require_trace: bool) -> None:
    expected_paths = [APP_BIN]
    if require_trace:
        expected_paths.append(TRACE_BIN)

    missing = [str(path) for path in expected_paths if not path.is_file()]
    if missing:
        raise RuntimeError(
            "Missing built binaries. Build the repo first: " + ", ".join(missing)
        )


def scene_id_from_pose_path(pose_path: Path) -> str:
    suffix = "_pose.json"
    pose_name = pose_path.name
    if not pose_name.endswith(suffix):
        raise RuntimeError(f"Unexpected pose filename: {pose_name}")
    return pose_name[: -len(suffix)]


def list_scene_ids() -> list[str]:
    pose_paths = sorted(CAN_BUS_DIR.glob("scene-*_pose.json"))
    return [scene_id_from_pose_path(path) for path in pose_paths]


def make_scene_paths(scene_id: str) -> tuple[Path, Path, Path]:
    return (
        CAN_BUS_DIR / f"{scene_id}_pose.json",
        CAN_BUS_DIR / f"{scene_id}_ms_imu.json",
        CAN_BUS_DIR / f"{scene_id}_zoe_veh_info.json",
    )


def load_pose_entries(scene_id: str) -> list[dict]:
    pose_path, imu_path, _ = make_scene_paths(scene_id)
    pose_entries = load_json(pose_path)
    imu_entries = load_json(imu_path)

    if not isinstance(pose_entries, list) or not isinstance(imu_entries, list):
        raise RuntimeError(f"Unexpected JSON root for {scene_id}")

    last_imu_utime = imu_entries[-1]["utime"]
    num_pose_after_last_imu = sum(
        1 for entry in pose_entries if entry["utime"] > last_imu_utime
    )
    if (
        num_pose_after_last_imu == 1
        and pose_entries
        and pose_entries[-1]["utime"] > last_imu_utime
    ):
        pose_entries = pose_entries[:-1]

    return pose_entries


def make_clean_dir(path: Path) -> None:
    if path.exists():
        shutil.rmtree(path)
    path.mkdir(parents=True)


def run_app_for_scene(scene_id: str, runs_root: Path) -> dict:
    run_dir = runs_root / scene_id
    make_clean_dir(run_dir)
    scenarios_dir = run_dir / "scenarios"
    scenarios_dir.mkdir()

    pose_path, imu_path, wheel_speed_path = make_scene_paths(scene_id)
    os.symlink(pose_path, scenarios_dir / pose_path.name)
    os.symlink(imu_path, scenarios_dir / imu_path.name)
    os.symlink(wheel_speed_path, scenarios_dir / wheel_speed_path.name)

    completed = subprocess.run(
        [str(APP_BIN)],
        cwd=run_dir,
        capture_output=True,
        text=True,
    )

    stdout_path = run_dir / "stdout.txt"
    stderr_path = run_dir / "stderr.txt"
    stdout_path.write_text(completed.stdout)
    stderr_path.write_text(completed.stderr)

    result = {
        "scene_id": scene_id,
        "exit_code": completed.returncode,
        "run_dir": str(run_dir),
        "stdout_path": str(stdout_path),
        "stderr_path": str(stderr_path),
    }

    if completed.returncode == 0:
        result["status"] = "completed"
        result.update(parse_run_summary(completed.stdout))
    else:
        result["status"] = "failed"
        failure_text = completed.stderr.strip() or completed.stdout.strip()
        result["failure_message"] = (
            failure_text.splitlines()[-1] if failure_text else "Run failed."
        )

    result_path = run_dir / "run_result.json"
    write_json(result_path, result)
    result["result_path"] = str(result_path)
    return result


def run_trace_for_scene(scene_id: str, traces_root: Path) -> dict:
    traces_root.mkdir(parents=True, exist_ok=True)
    pose_path, imu_path, wheel_speed_path = make_scene_paths(scene_id)
    trace_path = traces_root / f"{scene_id}_trace.json"

    subprocess.run(
        [
            str(TRACE_BIN),
            str(pose_path),
            str(imu_path),
            str(wheel_speed_path),
            str(trace_path),
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    trace = load_json(trace_path)
    if not isinstance(trace, dict):
        raise RuntimeError(f"Unexpected trace JSON root for {scene_id}")

    trace["trace_path"] = str(trace_path)
    return trace


def load_log_rows(run_dir: Path) -> list[dict[str, str]]:
    csv_path = run_dir / "output" / "eskf_sim_log.csv"
    with csv_path.open() as file:
        return list(csv.DictReader(file))


def git_output(args: list[str]) -> str:
    return subprocess.check_output(args, cwd=REPO_ROOT, text=True).strip()


def current_branch_name() -> str:
    return git_output(["git", "rev-parse", "--abbrev-ref", "HEAD"])


def current_commit_short_sha() -> str:
    return git_output(["git", "rev-parse", "--short", "HEAD"])
