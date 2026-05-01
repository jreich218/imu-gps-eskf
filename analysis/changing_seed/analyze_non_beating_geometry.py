#!/home/dfw/dev/python_laboratory/.venv/bin/python

from __future__ import annotations

import csv
import math
import os
import sys
from collections import Counter
from contextlib import contextmanager
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.gridspec import GridSpec


CHANGING_SEED_DIR = Path(__file__).resolve().parent
ANALYSIS_ROOT = CHANGING_SEED_DIR.parent
SCENE_ANALYSIS_DIR = ANALYSIS_ROOT / "scene_analysis"
if str(SCENE_ANALYSIS_DIR) not in sys.path:
    sys.path.insert(0, str(SCENE_ANALYSIS_DIR))

from common import (  # noqa: E402
    current_branch_name,
    current_commit_short_sha,
    ensure_binaries_exist,
    load_json,
    load_log_rows,
    load_pose_entries,
    make_clean_dir,
    run_app_for_scene,
    run_trace_for_scene,
    write_json,
)


GPS_SEED_ENV_VAR = "IMU_GPS_ESKF_GPS_SEED"
SOURCE_SWEEP_LABEL = current_commit_short_sha()
SOURCE_CASES_PATH = (
    ANALYSIS_ROOT / "supported_scene_seed_sweep" / SOURCE_SWEEP_LABEL / "non_beating_cases.csv"
)
RESULTS_LABEL = (
    f"{current_branch_name().replace('/', '_')}_{current_commit_short_sha()}"
    f"_supported_scene_seed_sweep_{SOURCE_SWEEP_LABEL}"
)
RESULTS_ROOT = CHANGING_SEED_DIR / "results" / RESULTS_LABEL / "non_beating_geometry"
APP_RUNS_ROOT = RESULTS_ROOT / "app_runs"
TRACES_ROOT = RESULTS_ROOT / "traces"
PLOTS_ROOT = RESULTS_ROOT / "plots"
CASES_CSV_PATH = RESULTS_ROOT / "cases.csv"
SUMMARY_MD_PATH = RESULTS_ROOT / "summary.md"
SUMMARY_JSON_PATH = RESULTS_ROOT / "summary.json"


def wrap_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def yaw_from_quaternion_wxyz(q_wxyz: list[float]) -> float:
    w, x, y, z = q_wxyz
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def unit_vec(yaw_rad: float) -> np.ndarray:
    return np.array([math.cos(yaw_rad), math.sin(yaw_rad)], dtype=float)


def left_vec(yaw_rad: float) -> np.ndarray:
    return np.array([-math.sin(yaw_rad), math.cos(yaw_rad)], dtype=float)


def handoff_frame_matrix(yaw_rad: float) -> np.ndarray:
    return np.array(
        [
            [math.cos(yaw_rad), math.sin(yaw_rad)],
            [-math.sin(yaw_rad), math.cos(yaw_rad)],
        ],
        dtype=float,
    )


@contextmanager
def gps_seed_env(seed: int):
    previous_value = os.environ.get(GPS_SEED_ENV_VAR)
    os.environ[GPS_SEED_ENV_VAR] = str(seed)
    try:
        yield
    finally:
        if previous_value is None:
            os.environ.pop(GPS_SEED_ENV_VAR, None)
        else:
            os.environ[GPS_SEED_ENV_VAR] = previous_value


def load_non_beating_cases() -> list[dict]:
    cases: list[dict] = []
    with SOURCE_CASES_PATH.open() as file:
        for row in csv.DictReader(file):
            cases.append(
                {
                    "seed": int(row["seed"]),
                    "scene_id": row["scene_id"],
                    "raw_gps_rmse_xy_m": float(row["raw_gps_rmse_xy_m"]),
                    "eskf_rmse_xy_m": float(row["eskf_rmse_xy_m"]),
                }
            )

    cases.sort(key=lambda case: (case["seed"], case["scene_id"]))
    return cases


def pose_arrays(pose_entries: list[dict]) -> dict[str, np.ndarray]:
    utime = np.array([int(entry["utime"]) for entry in pose_entries], dtype=np.int64)
    xy = np.array([entry["pos"][:2] for entry in pose_entries], dtype=float)
    yaw = np.unwrap(
        np.array(
            [yaw_from_quaternion_wxyz(entry["orientation"]) for entry in pose_entries],
            dtype=float,
        )
    )
    forward_speed = np.abs(
        np.array([float(entry["vel"][0]) for entry in pose_entries], dtype=float)
    )
    return {
        "utime": utime,
        "xy": xy,
        "yaw": yaw,
        "forward_speed": forward_speed,
    }


def interp_xy(utime: int, pose_data: dict[str, np.ndarray]) -> np.ndarray:
    utimes = pose_data["utime"]
    xy = pose_data["xy"]

    if utime <= int(utimes[0]):
        return xy[0].copy()
    if utime >= int(utimes[-1]):
        return xy[-1].copy()

    index = int(np.searchsorted(utimes, utime))
    if int(utimes[index]) == utime:
        return xy[index].copy()

    left_index = index - 1
    right_index = index
    left_utime = int(utimes[left_index])
    right_utime = int(utimes[right_index])
    alpha = (utime - left_utime) / float(right_utime - left_utime)
    return (1.0 - alpha) * xy[left_index] + alpha * xy[right_index]


def interp_scalar(utime: int, utimes: np.ndarray, values: np.ndarray) -> float:
    if utime <= int(utimes[0]):
        return float(values[0])
    if utime >= int(utimes[-1]):
        return float(values[-1])

    index = int(np.searchsorted(utimes, utime))
    if int(utimes[index]) == utime:
        return float(values[index])

    left_index = index - 1
    right_index = index
    left_utime = int(utimes[left_index])
    right_utime = int(utimes[right_index])
    alpha = (utime - left_utime) / float(right_utime - left_utime)
    return float((1.0 - alpha) * values[left_index] + alpha * values[right_index])


def interp_yaw(utime: int, pose_data: dict[str, np.ndarray]) -> float:
    return wrap_angle(interp_scalar(utime, pose_data["utime"], pose_data["yaw"]))


def pose_lookup_by_utime(pose_entries: list[dict]) -> dict[int, dict]:
    return {int(entry["utime"]): entry for entry in pose_entries}


def local_error_components(
    log_rows: list[dict[str, str]], pose_by_utime: dict[int, dict]
) -> dict[str, np.ndarray]:
    raw_along = []
    raw_cross = []
    eskf_along = []
    eskf_cross = []
    truth_xy = []

    for row in log_rows:
        utime = int(row["utime"])
        pose_entry = pose_by_utime[utime]
        yaw_rad = yaw_from_quaternion_wxyz(pose_entry["orientation"])
        fwd = unit_vec(yaw_rad)
        left = left_vec(yaw_rad)

        truth_point = np.array([float(row["true_x"]), float(row["true_y"])], dtype=float)
        gps_point = np.array([float(row["gps_x"]), float(row["gps_y"])], dtype=float)
        est_point = np.array([float(row["est_x"]), float(row["est_y"])], dtype=float)

        raw_error = gps_point - truth_point
        eskf_error = est_point - truth_point

        raw_along.append(float(raw_error.dot(fwd)))
        raw_cross.append(float(raw_error.dot(left)))
        eskf_along.append(float(eskf_error.dot(fwd)))
        eskf_cross.append(float(eskf_error.dot(left)))
        truth_xy.append(truth_point)

    truth_xy_array = np.array(truth_xy, dtype=float)
    arc_length = np.zeros(len(truth_xy_array), dtype=float)
    if len(truth_xy_array) >= 2:
        segment_lengths = np.linalg.norm(np.diff(truth_xy_array, axis=0), axis=1)
        arc_length[1:] = np.cumsum(segment_lengths)

    return {
        "truth_xy": truth_xy_array,
        "arc_length_m": arc_length,
        "raw_along_m": np.array(raw_along, dtype=float),
        "raw_cross_m": np.array(raw_cross, dtype=float),
        "eskf_along_m": np.array(eskf_along, dtype=float),
        "eskf_cross_m": np.array(eskf_cross, dtype=float),
    }


def case_metrics(
    case: dict, trace: dict, pose_entries: list[dict], log_rows: list[dict[str, str]]
) -> dict:
    pose_data = pose_arrays(pose_entries)
    pose_by_utime = pose_lookup_by_utime(pose_entries)

    ready_frame = trace["frames"][int(trace["ready_frame_index"])]
    startup_initialization = trace["startup_initialization"]
    handoff_utime = int(startup_initialization["previous_imu_utime"])
    handoff_xy = np.array(startup_initialization["p0_G"][:2], dtype=float)
    handoff_velocity_xy = np.array(startup_initialization["v0_G"][:2], dtype=float)
    handoff_speed_mps = float(np.linalg.norm(handoff_velocity_xy))
    handoff_yaw_rad = math.atan2(handoff_velocity_xy[1], handoff_velocity_xy[0])

    truth_handoff_xy = interp_xy(handoff_utime, pose_data)
    truth_handoff_yaw_rad = interp_yaw(handoff_utime, pose_data)
    truth_handoff_speed_mps = interp_scalar(
        handoff_utime, pose_data["utime"], pose_data["forward_speed"]
    )

    first_usable_gps_index = int(trace["first_usable_gps_index"])
    startup_begin_utime = int(trace["gps_samples"][first_usable_gps_index]["utime"])
    first_runtime_gps_index = int(startup_initialization["first_unprocessed_gps_index"])
    first_runtime_gps_utime = int(trace["gps_samples"][first_runtime_gps_index]["utime"])

    error_components = local_error_components(log_rows, pose_by_utime)
    raw_error_norm = np.hypot(
        error_components["raw_along_m"], error_components["raw_cross_m"]
    )
    eskf_error_norm = np.hypot(
        error_components["eskf_along_m"], error_components["eskf_cross_m"]
    )

    return {
        "seed": int(case["seed"]),
        "scene_id": str(case["scene_id"]),
        "startup_begin_utime": startup_begin_utime,
        "ready_utime": int(ready_frame["end_utime"]),
        "handoff_utime": handoff_utime,
        "first_runtime_gps_utime": first_runtime_gps_utime,
        "ready_gps_points_used": int(ready_frame["gps_points_used"]),
        "wheel_supported_travel_at_ready_m": float(
            ready_frame["wheel_supported_travel_m"]
        ),
        "raw_gps_rmse_xy_m": float(case["raw_gps_rmse_xy_m"]),
        "eskf_rmse_xy_m": float(case["eskf_rmse_xy_m"]),
        "rmse_ratio": float(case["eskf_rmse_xy_m"] / case["raw_gps_rmse_xy_m"]),
        "handoff_x_m": float(handoff_xy[0]),
        "handoff_y_m": float(handoff_xy[1]),
        "handoff_speed_mps": handoff_speed_mps,
        "truth_handoff_speed_mps": truth_handoff_speed_mps,
        "handoff_speed_error_mps": handoff_speed_mps - truth_handoff_speed_mps,
        "handoff_heading_error_deg": math.degrees(
            wrap_angle(handoff_yaw_rad - truth_handoff_yaw_rad)
        ),
        "handoff_position_error_m": float(np.linalg.norm(handoff_xy - truth_handoff_xy)),
        "mean_abs_raw_cross_m": float(
            np.mean(np.abs(error_components["raw_cross_m"]))
        ),
        "mean_abs_eskf_cross_m": float(
            np.mean(np.abs(error_components["eskf_cross_m"]))
        ),
        "mean_abs_raw_along_m": float(
            np.mean(np.abs(error_components["raw_along_m"]))
        ),
        "mean_abs_eskf_along_m": float(
            np.mean(np.abs(error_components["eskf_along_m"]))
        ),
        "max_raw_error_m": float(np.max(raw_error_norm)),
        "max_eskf_error_m": float(np.max(eskf_error_norm)),
        "gps_updates": len(log_rows),
    }


def plot_case(
    case: dict, trace: dict, pose_entries: list[dict], log_rows: list[dict[str, str]], metrics: dict
) -> Path:
    PLOTS_ROOT.mkdir(parents=True, exist_ok=True)

    pose_data = pose_arrays(pose_entries)
    pose_by_utime = pose_lookup_by_utime(pose_entries)
    error_components = local_error_components(log_rows, pose_by_utime)

    ready_frame = trace["frames"][int(trace["ready_frame_index"])]
    startup_initialization = trace["startup_initialization"]
    first_usable_gps_index = int(trace["first_usable_gps_index"])
    startup_begin_utime = metrics["startup_begin_utime"]
    first_runtime_gps_utime = metrics["first_runtime_gps_utime"]
    handoff_utime = metrics["handoff_utime"]

    startup_gps_used = trace["gps_samples"][
        first_usable_gps_index : ready_frame["fit_end_index"] + 1
    ]
    startup_gps_xy = np.array([[entry["x"], entry["y"]] for entry in startup_gps_used], dtype=float)
    startup_gps_utime = np.array([int(entry["utime"]) for entry in startup_gps_used], dtype=np.int64)
    startup_truth_xy = np.array(
        [pose_by_utime[int(utime)]["pos"][:2] for utime in startup_gps_utime], dtype=float
    )
    fitted_path_xy = np.array(ready_frame["path_xy"], dtype=float)

    startup_mask = pose_data["utime"] >= startup_begin_utime
    runtime_mask = pose_data["utime"] >= first_runtime_gps_utime
    startup_prefix_mask = (pose_data["utime"] >= startup_begin_utime) & (
        pose_data["utime"] <= int(ready_frame["end_utime"])
    )

    full_truth_xy = pose_data["xy"][startup_mask]
    runtime_truth_xy = pose_data["xy"][runtime_mask]
    startup_prefix_truth_xy = pose_data["xy"][startup_prefix_mask]

    run_gps_xy = np.array(
        [[float(row["gps_x"]), float(row["gps_y"])] for row in log_rows], dtype=float
    )
    est_xy = np.array(
        [[float(row["est_x"]), float(row["est_y"])] for row in log_rows], dtype=float
    )

    handoff_xy = np.array(startup_initialization["p0_G"][:2], dtype=float)
    handoff_velocity_xy = np.array(startup_initialization["v0_G"][:2], dtype=float)
    handoff_speed_mps = float(np.linalg.norm(handoff_velocity_xy))
    handoff_yaw_rad = math.atan2(handoff_velocity_xy[1], handoff_velocity_xy[0])
    truth_handoff_xy = interp_xy(handoff_utime, pose_data)
    truth_handoff_yaw_rad = interp_yaw(handoff_utime, pose_data)
    truth_handoff_speed_mps = metrics["truth_handoff_speed_mps"]

    transform = handoff_frame_matrix(handoff_yaw_rad)
    local_truth_xy = (transform @ (runtime_truth_xy - handoff_xy).T).T
    local_gps_xy = (transform @ (run_gps_xy - handoff_xy).T).T
    local_est_xy = (transform @ (est_xy - handoff_xy).T).T

    fig = plt.figure(figsize=(16, 13))
    grid = GridSpec(
        4,
        2,
        figure=fig,
        width_ratios=[1.15, 1.0],
        height_ratios=[1.0, 0.22, 1.0, 0.22],
    )

    startup_ax = fig.add_subplot(grid[0, 0])
    full_ax = fig.add_subplot(grid[0, 1])
    startup_info_ax = fig.add_subplot(grid[1, 0])
    full_info_ax = fig.add_subplot(grid[1, 1])
    local_ax = fig.add_subplot(grid[2, 0])
    error_ax = fig.add_subplot(grid[2, 1])
    local_info_ax = fig.add_subplot(grid[3, 0])
    error_info_ax = fig.add_subplot(grid[3, 1])

    for info_ax in (startup_info_ax, full_info_ax, local_info_ax, error_info_ax):
        info_ax.axis("off")

    startup_order = np.linspace(0.0, 1.0, len(startup_gps_xy))
    startup_scatter = startup_ax.scatter(
        startup_gps_xy[:, 0],
        startup_gps_xy[:, 1],
        c=startup_order,
        cmap="viridis",
        s=26,
        edgecolors="none",
        label="Startup GPS samples",
        zorder=3,
    )
    startup_ax.plot(
        startup_truth_xy[:, 0],
        startup_truth_xy[:, 1],
        color="#1f77b4",
        linewidth=2.2,
        label="Truth at startup GPS times",
        zorder=2,
    )
    startup_ax.plot(
        fitted_path_xy[:, 0],
        fitted_path_xy[:, 1],
        color="#d62728",
        linewidth=1.8,
        alpha=0.7,
        label="Fitted startup path",
        zorder=1,
    )
    startup_ax.scatter(
        [startup_truth_xy[0, 0]],
        [startup_truth_xy[0, 1]],
        color="#2ca02c",
        s=95,
        marker="o",
        label="Startup reference truth",
        zorder=4,
    )
    startup_ax.scatter(
        [startup_gps_xy[-1, 0]],
        [startup_gps_xy[-1, 1]],
        color="#ff7f0e",
        s=110,
        marker="D",
        label="Readiness GPS sample",
        zorder=5,
    )
    startup_ax.scatter(
        [ready_frame["truth_xy"][0]],
        [ready_frame["truth_xy"][1]],
        color="#1f77b4",
        s=120,
        marker="x",
        linewidths=2.0,
        label="Truth at readiness",
        zorder=5,
    )

    startup_arrow_origin = np.array(ready_frame["truth_xy"], dtype=float)
    arrow_length_m = min(8.0, max(3.0, 0.5 * handoff_speed_mps))
    for label, yaw_rad, color, width in [
        ("Selected heading", ready_frame["selected_yaw_rad"], "#d62728", 0.006),
        ("Truth heading", ready_frame["truth_yaw_rad"], "#1f77b4", 0.0055),
        ("Global heading", ready_frame["global_yaw_rad"], "#9467bd", 0.005),
    ]:
        direction = unit_vec(yaw_rad)
        startup_ax.quiver(
            [startup_arrow_origin[0]],
            [startup_arrow_origin[1]],
            [arrow_length_m * direction[0]],
            [arrow_length_m * direction[1]],
            angles="xy",
            scale_units="xy",
            scale=1,
            color=color,
            width=width,
            label=label,
            zorder=5,
        )

    startup_ax.set_title("Startup geometry")
    startup_ax.set_xlabel("x [m]")
    startup_ax.set_ylabel("y [m]")
    startup_ax.set_aspect("equal", adjustable="datalim")
    startup_ax.grid(True, alpha=0.25)
    fig.colorbar(startup_scatter, ax=startup_ax, fraction=0.046, pad=0.04, label="startup GPS order")
    startup_handles, startup_labels = startup_ax.get_legend_handles_labels()
    startup_info_ax.legend(
        startup_handles,
        startup_labels,
        loc="center left",
        fontsize=9,
        ncol=2,
        frameon=True,
    )
    startup_info_ax.text(
        0.98,
        0.5,
        (
            f"seed {metrics['seed']}\n"
            f"ready points: {metrics['ready_gps_points_used']}\n"
            f"wheel travel at ready: {metrics['wheel_supported_travel_at_ready_m']:.2f} m\n"
            f"handoff speed error: {metrics['handoff_speed_error_mps']:+.3f} m/s\n"
            f"handoff heading error: {metrics['handoff_heading_error_deg']:+.1f} deg"
        ),
        transform=startup_info_ax.transAxes,
        va="center",
        ha="right",
        fontsize=10,
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.9, "edgecolor": "#bbbbbb"},
    )

    full_ax.plot(
        full_truth_xy[:, 0],
        full_truth_xy[:, 1],
        color="#c7c7c7",
        linewidth=1.8,
        label="Truth from startup begin",
    )
    full_ax.plot(
        startup_prefix_truth_xy[:, 0],
        startup_prefix_truth_xy[:, 1],
        color="#1f77b4",
        linewidth=2.6,
        label="Startup prefix truth",
    )
    full_ax.plot(
        runtime_truth_xy[:, 0],
        runtime_truth_xy[:, 1],
        color="#08519c",
        linewidth=2.2,
        label="Post-startup truth",
    )
    full_ax.scatter(
        run_gps_xy[:, 0],
        run_gps_xy[:, 1],
        color="black",
        s=14,
        alpha=0.45,
        label="Raw GPS updates",
    )
    full_ax.plot(
        est_xy[:, 0],
        est_xy[:, 1],
        color="#d62728",
        linewidth=2.0,
        label="ESKF estimate",
    )
    full_ax.scatter(
        [handoff_xy[0]],
        [handoff_xy[1]],
        color="#ff7f0e",
        s=120,
        marker="D",
        label="Filter handoff state",
        zorder=5,
    )
    full_ax.scatter(
        [truth_handoff_xy[0]],
        [truth_handoff_xy[1]],
        color="#08519c",
        s=110,
        marker="x",
        linewidths=2.0,
        label="Truth at handoff",
        zorder=5,
    )

    velocity_arrow_scale_s = 0.4
    for label, origin_xy, velocity_xy, color, width in [
        ("Handed velocity", handoff_xy, handoff_velocity_xy, "#d62728", 0.006),
        (
            "Truth velocity",
            truth_handoff_xy,
            truth_handoff_speed_mps * unit_vec(truth_handoff_yaw_rad),
            "#08519c",
            0.005,
        ),
    ]:
        full_ax.quiver(
            [origin_xy[0]],
            [origin_xy[1]],
            [velocity_arrow_scale_s * velocity_xy[0]],
            [velocity_arrow_scale_s * velocity_xy[1]],
            angles="xy",
            scale_units="xy",
            scale=1,
            color=color,
            width=width,
            label=label,
            zorder=5,
        )

    full_ax.set_title("Full post-startup geometry in world frame")
    full_ax.set_xlabel("x [m]")
    full_ax.set_ylabel("y [m]")
    full_ax.set_aspect("equal", adjustable="datalim")
    full_ax.grid(True, alpha=0.25)
    full_handles, full_labels = full_ax.get_legend_handles_labels()
    full_info_ax.legend(
        full_handles,
        full_labels,
        loc="center left",
        fontsize=9,
        ncol=2,
        frameon=True,
    )
    full_info_ax.text(
        0.98,
        0.5,
        (
            f"raw RMSE: {metrics['raw_gps_rmse_xy_m']:.3f} m\n"
            f"ESKF RMSE: {metrics['eskf_rmse_xy_m']:.3f} m\n"
            f"ratio: {metrics['rmse_ratio']:.3f}\n"
            f"updates: {metrics['gps_updates']}"
        ),
        transform=full_info_ax.transAxes,
        va="center",
        ha="right",
        fontsize=10,
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.9, "edgecolor": "#bbbbbb"},
    )

    local_ax.plot(
        local_truth_xy[:, 0],
        local_truth_xy[:, 1],
        color="#08519c",
        linewidth=2.2,
        label="Truth",
    )
    local_ax.scatter(
        local_gps_xy[:, 0],
        local_gps_xy[:, 1],
        color="black",
        s=14,
        alpha=0.45,
        label="Raw GPS",
    )
    local_ax.plot(
        local_est_xy[:, 0],
        local_est_xy[:, 1],
        color="#d62728",
        linewidth=2.0,
        label="ESKF",
    )
    local_ax.scatter(
        [0.0],
        [0.0],
        color="#ff7f0e",
        s=120,
        marker="D",
        label="Handoff origin",
        zorder=5,
    )
    local_ax.axhline(0.0, color="#bbbbbb", linewidth=1.0, linestyle="--")
    local_ax.axvline(0.0, color="#bbbbbb", linewidth=1.0, linestyle="--")
    local_ax.set_title("Post-startup run in handoff frame")
    local_ax.set_xlabel("along handed heading [m]")
    local_ax.set_ylabel("left of handed heading [m]")
    local_ax.set_aspect("equal", adjustable="datalim")
    local_ax.grid(True, alpha=0.25)
    local_handles, local_labels = local_ax.get_legend_handles_labels()
    local_info_ax.legend(
        local_handles,
        local_labels,
        loc="center",
        fontsize=9,
        ncol=2,
        frameon=True,
    )

    error_ax.scatter(
        error_components["raw_along_m"],
        error_components["raw_cross_m"],
        color="black",
        s=20,
        alpha=0.45,
        label="Raw GPS error",
    )
    error_ax.scatter(
        error_components["eskf_along_m"],
        error_components["eskf_cross_m"],
        color="#d62728",
        s=20,
        alpha=0.45,
        label="ESKF error",
    )
    error_ax.axhline(0.0, color="#bbbbbb", linewidth=1.0, linestyle="--")
    error_ax.axvline(0.0, color="#bbbbbb", linewidth=1.0, linestyle="--")
    error_ax.set_title("Error geometry in local truth frames")
    error_ax.set_xlabel("along-track error [m]")
    error_ax.set_ylabel("cross-track error [m]")
    error_ax.set_aspect("equal", adjustable="datalim")
    error_ax.grid(True, alpha=0.25)
    error_handles, error_labels = error_ax.get_legend_handles_labels()
    error_info_ax.legend(
        error_handles,
        error_labels,
        loc="center left",
        fontsize=9,
        ncol=2,
        frameon=True,
    )
    error_info_ax.text(
        0.98,
        0.5,
        (
            f"mean |raw cross|: {metrics['mean_abs_raw_cross_m']:.2f} m\n"
            f"mean |ESKF cross|: {metrics['mean_abs_eskf_cross_m']:.2f} m\n"
            f"mean |raw along|: {metrics['mean_abs_raw_along_m']:.2f} m\n"
            f"mean |ESKF along|: {metrics['mean_abs_eskf_along_m']:.2f} m"
        ),
        transform=error_info_ax.transAxes,
        va="center",
        ha="right",
        fontsize=10,
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.9, "edgecolor": "#bbbbbb"},
    )

    fig.suptitle(
        f"{case['scene_id']} seed {case['seed']}: non-beating geometry workup",
        fontsize=16,
        y=0.98,
    )
    fig.tight_layout(rect=(0.0, 0.02, 1.0, 0.96), h_pad=2.2, w_pad=2.0)

    plot_path = PLOTS_ROOT / f"seed_{case['seed']:03d}_{case['scene_id']}.png"
    fig.savefig(plot_path, dpi=180)
    plt.close(fig)
    return plot_path


def write_cases_csv(case_summaries: list[dict]) -> None:
    if not case_summaries:
        return

    fieldnames = [
        "seed",
        "scene_id",
        "raw_gps_rmse_xy_m",
        "eskf_rmse_xy_m",
        "rmse_ratio",
        "handoff_speed_mps",
        "truth_handoff_speed_mps",
        "handoff_speed_error_mps",
        "handoff_heading_error_deg",
        "handoff_position_error_m",
        "ready_gps_points_used",
        "wheel_supported_travel_at_ready_m",
        "mean_abs_raw_cross_m",
        "mean_abs_eskf_cross_m",
        "mean_abs_raw_along_m",
        "mean_abs_eskf_along_m",
        "max_raw_error_m",
        "max_eskf_error_m",
        "gps_updates",
        "plot_path",
        "trace_path",
        "run_result_path",
    ]

    with CASES_CSV_PATH.open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        for case_summary in case_summaries:
            writer.writerow({name: case_summary[name] for name in fieldnames})


def write_summary_md(case_summaries: list[dict]) -> None:
    scene_counts = Counter(case_summary["scene_id"] for case_summary in case_summaries)
    worst_cases = sorted(
        case_summaries, key=lambda case_summary: case_summary["rmse_ratio"], reverse=True
    )[:10]
    largest_heading_error = sorted(
        case_summaries,
        key=lambda case_summary: abs(case_summary["handoff_heading_error_deg"]),
        reverse=True,
    )[:10]

    lines = [
        "# Non-beating geometry workup",
        "",
        f"- source cases: `{SOURCE_CASES_PATH}`",
        f"- non-beating scene-seed instances: `{len(case_summaries)}`",
        f"- unique scenes: `{len(scene_counts)}`",
        "",
        "## Scenes by recurrence",
        "",
        "| scene | count |",
        "| --- | ---: |",
    ]

    for scene_id, count in scene_counts.most_common():
        lines.append(f"| {scene_id} | {count} |")

    lines.extend(
        [
            "",
            "## Worst RMSE ratios",
            "",
            "| seed | scene | raw RMSE [m] | ESKF RMSE [m] | ratio | heading error [deg] | speed error [m/s] | plot |",
            "| ---: | --- | ---: | ---: | ---: | ---: | ---: | --- |",
        ]
    )
    for case_summary in worst_cases:
        lines.append(
            "| "
            f"{case_summary['seed']} | "
            f"{case_summary['scene_id']} | "
            f"{case_summary['raw_gps_rmse_xy_m']:.5f} | "
            f"{case_summary['eskf_rmse_xy_m']:.5f} | "
            f"{case_summary['rmse_ratio']:.5f} | "
            f"{case_summary['handoff_heading_error_deg']:+.2f} | "
            f"{case_summary['handoff_speed_error_mps']:+.4f} | "
            f"`{Path(case_summary['plot_path']).name}` |"
        )

    lines.extend(
        [
            "",
            "## Largest handoff heading errors",
            "",
            "| seed | scene | heading error [deg] | raw RMSE [m] | ESKF RMSE [m] | plot |",
            "| ---: | --- | ---: | ---: | ---: | --- |",
        ]
    )
    for case_summary in largest_heading_error:
        lines.append(
            "| "
            f"{case_summary['seed']} | "
            f"{case_summary['scene_id']} | "
            f"{case_summary['handoff_heading_error_deg']:+.2f} | "
            f"{case_summary['raw_gps_rmse_xy_m']:.5f} | "
            f"{case_summary['eskf_rmse_xy_m']:.5f} | "
            f"`{Path(case_summary['plot_path']).name}` |"
        )

    SUMMARY_MD_PATH.write_text("\n".join(lines) + "\n")


def main() -> None:
    ensure_binaries_exist(require_trace=True)
    if not SOURCE_CASES_PATH.is_file():
        raise RuntimeError(f"Missing case list: {SOURCE_CASES_PATH}")

    cases = load_non_beating_cases()
    make_clean_dir(RESULTS_ROOT)

    case_summaries = []
    for case in cases:
        seed_runs_root = APP_RUNS_ROOT / f"seed_{case['seed']:03d}"
        seed_traces_root = TRACES_ROOT / f"seed_{case['seed']:03d}"
        with gps_seed_env(int(case["seed"])):
            app_result = run_app_for_scene(case["scene_id"], seed_runs_root)
            if app_result["status"] != "completed":
                raise RuntimeError(
                    f"{case['scene_id']} failed while reproducing seed {case['seed']}."
                )
            trace = run_trace_for_scene(case["scene_id"], seed_traces_root)

        pose_entries = load_pose_entries(case["scene_id"])
        log_rows = load_log_rows(Path(app_result["run_dir"]))
        metrics = case_metrics(case, trace, pose_entries, log_rows)
        plot_path = plot_case(case, trace, pose_entries, log_rows, metrics)

        case_summary = {
            **metrics,
            "plot_path": str(plot_path),
            "trace_path": str(trace["trace_path"]),
            "run_result_path": str(app_result["result_path"]),
        }
        case_summaries.append(case_summary)
        print(f"Wrote geometry workup for {case['scene_id']} seed {case['seed']}")

    write_cases_csv(case_summaries)
    write_summary_md(case_summaries)
    write_json(
        SUMMARY_JSON_PATH,
        {
            "source_cases_path": str(SOURCE_CASES_PATH),
            "source_sweep_label": SOURCE_SWEEP_LABEL,
            "case_count": len(case_summaries),
            "cases": case_summaries,
        },
    )
    print(f"Wrote {SUMMARY_JSON_PATH}")


if __name__ == "__main__":
    main()
