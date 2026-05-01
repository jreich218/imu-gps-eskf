#!/usr/bin/env python3

from __future__ import annotations

import concurrent.futures
import math
import os
import shutil
import statistics
import sys
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.gridspec import GridSpec
from matplotlib.patches import Circle


SCENE_ANALYSIS_DIR = Path(__file__).resolve().parents[1]
if str(SCENE_ANALYSIS_DIR) not in sys.path:
    sys.path.insert(0, str(SCENE_ANALYSIS_DIR))

from common import (  # noqa: E402
    current_branch_name,
    current_commit_short_sha,
    current_run_label,
    ensure_binaries_exist,
    list_scene_ids,
    load_log_rows,
    load_pose_entries,
    make_clean_dir,
    run_app_for_scene,
    run_trace_for_scene,
    write_json,
)


RUN_LABEL = current_run_label()
WORKER_COUNT = max(1, int(os.cpu_count() or 1))
RESULTS_ROOT = Path(__file__).resolve().parent / "results" / RUN_LABEL
SCRATCH_ROOT = Path(__file__).resolve().parent / "scratch" / RUN_LABEL
APP_RUNS_ROOT = SCRATCH_ROOT / "app_runs"
TRACES_ROOT = SCRATCH_ROOT / "traces"
TRACE_OVERVIEW_PLOTS_ROOT = RESULTS_ROOT / "trace_overview_plots"
READINESS_PLOTS_ROOT = RESULTS_ROOT / "readiness_plots"
HANDOFF_PLOTS_ROOT = RESULTS_ROOT / "handoff_plots"
DEFAULT_GPS_SEED = 1234
GPS_SIGMA_M = 2.0
EARLY_GATE_M = 10.0
LATE_GATE_M = 7.0
RELAXED_GATE_POINT_COUNT = 80


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


def truth_xy_by_utime(pose_entries: list[dict]) -> dict[int, np.ndarray]:
    return {
        int(entry["utime"]): np.array(entry["pos"][:2], dtype=float)
        for entry in pose_entries
    }


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


def compute_pca_axis(points_xy: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    mean_xy = points_xy.mean(axis=0)
    centered = points_xy - mean_xy
    covariance = centered.T @ centered / float(len(points_xy))
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    axis = eigenvectors[:, np.argmax(eigenvalues)]
    start_to_end = points_xy[-1] - points_xy[0]
    if np.dot(axis, start_to_end) < 0.0:
        axis = -axis
    return mean_xy, axis


def safe_max(values: np.ndarray) -> float | None:
    if values.size == 0:
        return None
    return float(values.max())


def format_optional(value: float | None, fmt: str) -> str:
    if value is None:
        return "n/a"
    return format(value, fmt)


def format_optional_m(value: float | None) -> str:
    if value is None:
        return "n/a"
    return f"{value:.2f} m"


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


def summarize_log_rows(log_rows: list[dict[str, str]]) -> dict:
    raw_error_norm = []
    eskf_error_norm = []

    for row in log_rows:
        gps_x = float(row["gps_x"])
        gps_y = float(row["gps_y"])
        true_x = float(row["true_x"])
        true_y = float(row["true_y"])
        raw_error_norm.append(math.hypot(gps_x - true_x, gps_y - true_y))
        eskf_error_norm.append(float(row["err_norm"]))

    return {
        "num_updates": len(log_rows),
        "raw_error_norm": raw_error_norm,
        "eskf_error_norm": eskf_error_norm,
        "eskf_beats_raw_count": sum(
            eskf < raw for eskf, raw in zip(eskf_error_norm, raw_error_norm)
        ),
    }


def compute_trace_overview_metrics(trace: dict, pose_entries: list[dict]) -> dict:
    frames = trace["frames"]
    gps_samples = trace["gps_samples"]
    first_usable_index = int(trace["first_usable_gps_index"])
    first_usable_utime = int(gps_samples[first_usable_index]["utime"])
    truth_by_time = truth_xy_by_utime(pose_entries)
    origin_truth_xy = truth_by_time[first_usable_utime]

    gps_used_xy = np.array(
        [[sample["x"], sample["y"]] for sample in gps_samples[first_usable_index:]],
        dtype=float,
    )
    gps_used_utimes = np.array(
        [int(sample["utime"]) for sample in gps_samples[first_usable_index:]],
        dtype=np.int64,
    )
    truth_used_xy = np.array(
        [truth_by_time[int(utime)] for utime in gps_used_utimes], dtype=float
    )

    projected_separation = np.array(
        [frame["projected_separation_m"] for frame in frames], dtype=float
    )
    gps_points_used = np.array(
        [frame["gps_points_used"] for frame in frames], dtype=int
    )
    post_relax_mask = gps_points_used >= RELAXED_GATE_POINT_COUNT

    gps_radius = np.linalg.norm(gps_used_xy - origin_truth_xy[None, :], axis=1)
    truth_radius = np.linalg.norm(truth_used_xy - origin_truth_xy[None, :], axis=1)

    return {
        "scene_id": str(trace["scene"]),
        "frame_count": len(frames),
        "ready_frame_index": trace["ready_frame_index"],
        "gps_points_used_final": int(gps_points_used[-1]),
        "max_projected_separation_m": float(projected_separation.max()),
        "max_projected_pre80_m": safe_max(projected_separation[~post_relax_mask]),
        "max_projected_post80_m": safe_max(projected_separation[post_relax_mask]),
        "final_projected_separation_m": float(projected_separation[-1]),
        "max_wheel_supported_travel_m": float(
            max(frame["wheel_supported_travel_m"] for frame in frames)
        ),
        "truth_max_radius_m": float(truth_radius.max()),
        "truth_final_radius_m": float(truth_radius[-1]),
        "gps_max_radius_m": float(gps_radius.max()),
        "gps_final_radius_m": float(gps_radius[-1]),
    }


def plot_trace_overview(
    scene_id: str, trace: dict, pose_entries: list[dict], metrics: dict
) -> Path:
    TRACE_OVERVIEW_PLOTS_ROOT.mkdir(parents=True, exist_ok=True)

    frames = trace["frames"]
    gps_samples = trace["gps_samples"]
    first_usable_index = int(trace["first_usable_gps_index"])

    gps_used_xy = np.array(
        [[sample["x"], sample["y"]] for sample in gps_samples[first_usable_index:]],
        dtype=float,
    )
    gps_points_used = np.array([frame["gps_points_used"] for frame in frames], dtype=int)
    projected_separation = np.array(
        [frame["projected_separation_m"] for frame in frames], dtype=float
    )
    wheel_supported_travel = np.array(
        [frame["wheel_supported_travel_m"] for frame in frames], dtype=float
    )
    required_wheel_supported_travel = np.array(
        [frame["required_wheel_supported_travel_m"] for frame in frames], dtype=float
    )
    truth_by_time = truth_xy_by_utime(pose_entries)
    first_usable_utime = int(gps_samples[first_usable_index]["utime"])
    origin_truth_xy = truth_by_time[first_usable_utime]
    truth_path_xy = np.array(
        [
            entry["pos"][:2]
            for entry in pose_entries
            if int(entry["utime"]) >= first_usable_utime
        ],
        dtype=float,
    )
    gps_frame_xy = np.array([frame["latest_gps_xy"] for frame in frames], dtype=float)
    truth_frame_xy = np.array([frame["truth_xy"] for frame in frames], dtype=float)
    gps_radius = np.linalg.norm(gps_frame_xy - origin_truth_xy[None, :], axis=1)
    truth_radius = np.linalg.norm(truth_frame_xy - origin_truth_xy[None, :], axis=1)

    mean_xy, axis_xy = compute_pca_axis(gps_used_xy)
    axis_half_length = max(
        10.0,
        0.55 * np.ptp(gps_used_xy[:, 0]) + 0.55 * np.ptp(gps_used_xy[:, 1]),
    )
    axis_line = np.vstack(
        [mean_xy - axis_half_length * axis_xy, mean_xy + axis_half_length * axis_xy]
    )

    fig = plt.figure(figsize=(16, 9))
    grid = GridSpec(
        2, 2, figure=fig, width_ratios=[1.35, 1.0], height_ratios=[1.0, 1.0]
    )

    full_ax = fig.add_subplot(grid[:, 0])
    zoom_ax = fig.add_subplot(grid[0, 1])
    metric_ax = fig.add_subplot(grid[1, 1])

    color_values = np.linspace(0.0, 1.0, len(gps_used_xy))
    scatter = full_ax.scatter(
        gps_used_xy[:, 0],
        gps_used_xy[:, 1],
        c=color_values,
        cmap="viridis",
        s=26,
        edgecolors="none",
        zorder=3,
    )
    full_ax.plot(
        truth_path_xy[:, 0],
        truth_path_xy[:, 1],
        color="#4C72B0",
        linewidth=2.0,
        alpha=0.7,
        zorder=2,
        label="Truth path",
    )
    full_ax.scatter(
        [gps_used_xy[0, 0]],
        [gps_used_xy[0, 1]],
        color="#ff7f0e",
        s=122,
        marker="o",
        zorder=4,
        label="First usable GPS",
    )
    full_ax.scatter(
        [gps_used_xy[-1, 0]],
        [gps_used_xy[-1, 1]],
        color="#ff7f0e",
        s=89,
        marker="D",
        zorder=4,
        label="Last GPS",
    )
    full_ax.plot(
        axis_line[:, 0],
        axis_line[:, 1],
        color="black",
        linestyle="--",
        linewidth=1.4,
        alpha=0.6,
        zorder=1,
        label="Final PCA axis",
    )
    full_ax.set_title("Startup trace in world frame")
    full_ax.set_xlabel("x [m]")
    full_ax.set_ylabel("y [m]")
    full_ax.set_aspect("equal", adjustable="box")
    full_ax.grid(True, alpha=0.25)
    full_ax.legend(loc="best")
    fig.colorbar(scatter, ax=full_ax, fraction=0.046, pad=0.04, label="GPS order")

    zoom_ax.plot(
        truth_path_xy[:, 0],
        truth_path_xy[:, 1],
        color="#4C72B0",
        linewidth=2.0,
        alpha=0.7,
        zorder=2,
    )
    zoom_ax.scatter(
        gps_used_xy[:, 0],
        gps_used_xy[:, 1],
        c=color_values,
        cmap="viridis",
        s=28,
        edgecolors="none",
        zorder=3,
    )
    zoom_ax.scatter(
        [origin_truth_xy[0]],
        [origin_truth_xy[1]],
        color="#d62728",
        s=100,
        marker="x",
        linewidths=2.2,
        zorder=5,
    )
    for radius_m, color, linestyle, alpha in [
        (GPS_SIGMA_M, "#6baed6", "-", 0.8),
        (2.0 * GPS_SIGMA_M, "#3182bd", "-", 0.8),
        (LATE_GATE_M, "#31a354", "--", 0.7),
        (EARLY_GATE_M, "#de2d26", "--", 0.6),
    ]:
        zoom_ax.add_patch(
            Circle(
                (origin_truth_xy[0], origin_truth_xy[1]),
                radius_m,
                fill=False,
                color=color,
                linestyle=linestyle,
                linewidth=1.5,
                alpha=alpha,
                zorder=1,
            )
        )
    zoom_span = max(EARLY_GATE_M + 2.5, metrics["gps_max_radius_m"] + 1.0)
    zoom_ax.set_xlim(origin_truth_xy[0] - zoom_span, origin_truth_xy[0] + zoom_span)
    zoom_ax.set_ylim(origin_truth_xy[1] - zoom_span, origin_truth_xy[1] + zoom_span)
    zoom_ax.set_title("Start-centered view")
    zoom_ax.set_xlabel("x [m]")
    zoom_ax.set_ylabel("y [m]")
    zoom_ax.set_aspect("equal", adjustable="box")
    zoom_ax.grid(True, alpha=0.25)

    metric_ax.plot(
        gps_points_used,
        projected_separation,
        color="black",
        linewidth=2.0,
        label="Projected separation",
    )
    metric_ax.plot(
        gps_points_used,
        wheel_supported_travel,
        color="#d62728",
        linewidth=2.0,
        label="Wheel-supported travel",
    )
    metric_ax.step(
        gps_points_used,
        required_wheel_supported_travel,
        where="post",
        color="#ff7f0e",
        linewidth=1.8,
        linestyle="--",
        label="Required wheel-supported travel",
    )
    metric_ax.plot(
        gps_points_used,
        gps_radius,
        color="#2ca02c",
        linewidth=1.6,
        linestyle=":",
        label="GPS radius from start truth",
    )
    metric_ax.plot(
        gps_points_used,
        truth_radius,
        color="#4C72B0",
        linewidth=1.8,
        linestyle="--",
        label="Truth radius from start truth",
    )
    metric_ax.axvline(
        RELAXED_GATE_POINT_COUNT,
        color="#7f7f7f",
        linewidth=1.2,
        linestyle="--",
        alpha=0.8,
    )
    metric_ax.set_title("Startup-separation history")
    metric_ax.set_xlabel("GPS points used")
    metric_ax.set_ylabel("meters")
    metric_ax.grid(True, alpha=0.25)
    metric_ax.legend(loc="upper right")

    ready_frame_text = (
        "none"
        if metrics["ready_frame_index"] is None
        else str(metrics["ready_frame_index"])
    )
    info_lines = [
        f"ready frame: {ready_frame_text}",
        f"truth max radius: {metrics['truth_max_radius_m']:.2f} m",
        f"GPS max radius: {metrics['gps_max_radius_m']:.2f} m",
        f"max projected <80: {format_optional_m(metrics['max_projected_pre80_m'])}",
        f"max projected >=80: {format_optional_m(metrics['max_projected_post80_m'])}",
        f"max wheel travel: {metrics['max_wheel_supported_travel_m']:.2f} m",
        f"final projected: {metrics['final_projected_separation_m']:.2f} m",
    ]
    metric_ax.text(
        0.02,
        0.98,
        "\n".join(info_lines),
        transform=metric_ax.transAxes,
        va="top",
        ha="left",
        fontsize=10,
        bbox={
            "boxstyle": "round",
            "facecolor": "white",
            "alpha": 0.9,
            "edgecolor": "#bbbbbb",
        },
    )

    fig.suptitle(f"{scene_id}: startup trace overview", fontsize=15)
    fig.tight_layout()

    plot_path = TRACE_OVERVIEW_PLOTS_ROOT / f"{scene_id}_startup_trace_overview.png"
    fig.savefig(plot_path, dpi=180)
    plt.close(fig)
    return plot_path


def compute_readiness_metrics(
    trace: dict, pose_entries: list[dict], app_result: dict
) -> dict:
    ready_index = int(trace["ready_frame_index"])
    frames = trace["frames"]
    ready_frame = frames[ready_index]
    previous_frame = frames[ready_index - 1] if ready_index > 0 else ready_frame
    first_usable_index = int(trace["first_usable_gps_index"])
    first_usable_utime = int(trace["gps_samples"][first_usable_index]["utime"])
    truth_by_time = truth_xy_by_utime(pose_entries)
    origin_truth_xy = truth_by_time[first_usable_utime]
    ready_truth_xy = np.array(ready_frame["truth_xy"], dtype=float)
    truth_radius_at_ready_m = float(np.linalg.norm(ready_truth_xy - origin_truth_xy))

    wrapped_yaw_error_deg = math.degrees(
        wrap_angle(ready_frame["selected_yaw_rad"] - ready_frame["truth_yaw_rad"])
    )
    truth_speed_mps = float(ready_frame["truth_speed_mps"])
    wheel_speed_mps = float(ready_frame["wheel_speed_mps"])
    speed_ratio = (
        wheel_speed_mps / truth_speed_mps if truth_speed_mps > 0.0 else None
    )

    return {
        "ready_frame_index": ready_index,
        "ready_gps_points_used": int(ready_frame["gps_points_used"]),
        "ready_utime": int(ready_frame["end_utime"]),
        "startup_duration_s": (int(ready_frame["end_utime"]) - first_usable_utime)
        * 1e-6,
        "truth_radius_at_ready_m": truth_radius_at_ready_m,
        "ready_projected_separation_m": float(ready_frame["projected_separation_m"]),
        "ready_wheel_supported_travel_m": float(
            ready_frame["wheel_supported_travel_m"]
        ),
        "ready_required_wheel_supported_travel_m": float(
            ready_frame["required_wheel_supported_travel_m"]
        ),
        "selected_yaw_deg": math.degrees(ready_frame["selected_yaw_rad"]),
        "truth_yaw_deg": math.degrees(ready_frame["truth_yaw_rad"]),
        "wrapped_yaw_error_deg": wrapped_yaw_error_deg,
        "wheel_speed_mps": wheel_speed_mps,
        "truth_speed_mps": truth_speed_mps,
        "speed_ratio": speed_ratio,
        "prev_projected_separation_m": float(previous_frame["projected_separation_m"]),
        "prev_selected_yaw_deg": math.degrees(previous_frame["selected_yaw_rad"]),
        "ready_selected_yaw_deg": math.degrees(ready_frame["selected_yaw_rad"]),
        "raw_rmse_xy_m": float(app_result["raw_gps_rmse_xy"]),
        "eskf_rmse_xy_m": float(app_result["eskf_rmse_xy"]),
        "gps_updates": int(app_result["gps_updates"]),
    }


def plot_readiness_overview(
    scene_id: str,
    trace: dict,
    pose_entries: list[dict],
    log_rows: list[dict[str, str]],
    metrics: dict,
) -> Path:
    READINESS_PLOTS_ROOT.mkdir(parents=True, exist_ok=True)

    frames = trace["frames"]
    ready_index = int(trace["ready_frame_index"])
    ready_frame = frames[ready_index]
    first_usable_index = int(trace["first_usable_gps_index"])
    first_usable_utime = int(trace["gps_samples"][first_usable_index]["utime"])

    truth_by_time = truth_xy_by_utime(pose_entries)
    origin_truth_xy = truth_by_time[first_usable_utime]

    gps_so_far = trace["gps_samples"][first_usable_index : ready_frame["fit_end_index"] + 1]
    gps_so_far_xy = np.array([[p["x"], p["y"]] for p in gps_so_far], dtype=float)
    gps_so_far_utimes = np.array([int(p["utime"]) for p in gps_so_far], dtype=np.int64)
    truth_so_far_xy = np.array(
        [truth_by_time[int(utime)] for utime in gps_so_far_utimes], dtype=float
    )
    color_values = np.linspace(0.0, 1.0, len(gps_so_far_xy))
    path_xy = np.array(ready_frame["path_xy"], dtype=float)
    fitted_xy = np.array(ready_frame["fitted_endpoint_xy"], dtype=float)
    ready_truth_xy = np.array(ready_frame["truth_xy"], dtype=float)

    gps_points_used = np.array([frame["gps_points_used"] for frame in frames], dtype=int)
    projected_separation = np.array(
        [frame["projected_separation_m"] for frame in frames], dtype=float
    )
    wheel_supported_travel = np.array(
        [frame["wheel_supported_travel_m"] for frame in frames], dtype=float
    )
    required_wheel_supported_travel = np.array(
        [frame["required_wheel_supported_travel_m"] for frame in frames], dtype=float
    )
    truth_frame_xy = np.array([frame["truth_xy"] for frame in frames], dtype=float)
    gps_frame_xy = np.array([frame["latest_gps_xy"] for frame in frames], dtype=float)
    truth_radius = np.linalg.norm(truth_frame_xy - origin_truth_xy[None, :], axis=1)
    gps_radius = np.linalg.norm(gps_frame_xy - origin_truth_xy[None, :], axis=1)

    est_xy = np.array(
        [[float(row["est_x"]), float(row["est_y"])] for row in log_rows], dtype=float
    )
    run_gps_xy = np.array(
        [[float(row["gps_x"]), float(row["gps_y"])] for row in log_rows], dtype=float
    )
    run_truth_xy = np.array(
        [[float(row["true_x"]), float(row["true_y"])] for row in log_rows], dtype=float
    )
    log_summary = summarize_log_rows(log_rows)
    raw_error_norm = np.array(log_summary["raw_error_norm"], dtype=float)
    eskf_error_norm = np.array(log_summary["eskf_error_norm"], dtype=float)

    fig = plt.figure(figsize=(16, 10))
    grid = GridSpec(
        2, 2, figure=fig, width_ratios=[1.25, 1.0], height_ratios=[1.0, 1.0]
    )

    startup_ax = fig.add_subplot(grid[0, 0])
    history_ax = fig.add_subplot(grid[0, 1])
    traj_ax = fig.add_subplot(grid[1, 0])
    err_ax = fig.add_subplot(grid[1, 1])

    scatter = startup_ax.scatter(
        gps_so_far_xy[:, 0],
        gps_so_far_xy[:, 1],
        c=color_values,
        cmap="viridis",
        s=28,
        edgecolors="none",
        zorder=3,
        label="GPS used by startup",
    )
    startup_ax.plot(
        truth_so_far_xy[:, 0],
        truth_so_far_xy[:, 1],
        color="#4C72B0",
        linewidth=2.0,
        alpha=0.8,
        zorder=2,
        label="Truth path so far",
    )
    startup_ax.plot(
        path_xy[:, 0],
        path_xy[:, 1],
        color="#d62728",
        linewidth=1.8,
        alpha=0.5,
        zorder=1,
        label="Fitted startup path",
    )
    startup_ax.scatter(
        [origin_truth_xy[0]],
        [origin_truth_xy[1]],
        color="#2ca02c",
        s=90,
        marker="o",
        zorder=4,
        label="Start truth",
    )
    startup_ax.scatter(
        [gps_so_far_xy[-1, 0]],
        [gps_so_far_xy[-1, 1]],
        color="#ff7f0e",
        s=120,
        marker="D",
        zorder=5,
        label="Readiness GPS sample",
    )
    startup_ax.scatter(
        [ready_truth_xy[0]],
        [ready_truth_xy[1]],
        color="#1f77b4",
        s=110,
        marker="x",
        linewidths=2.0,
        zorder=5,
        label="Truth at readiness",
    )

    arrow_length_m = 3.0
    for label, yaw_rad, color, width in [
        ("Selected yaw", ready_frame["selected_yaw_rad"], "#d62728", 0.006),
        ("Global yaw", ready_frame["global_yaw_rad"], "#9467bd", 0.005),
        ("Raw tangent", ready_frame["fitted_yaw_rad"], "#ff9896", 0.004),
        ("Truth tangent", ready_frame["truth_yaw_rad"], "#1f77b4", 0.0055),
    ]:
        direction = unit_vec(yaw_rad)
        startup_ax.quiver(
            [fitted_xy[0]],
            [fitted_xy[1]],
            [arrow_length_m * direction[0]],
            [arrow_length_m * direction[1]],
            angles="xy",
            scale_units="xy",
            scale=1,
            color=color,
            width=width,
            zorder=5,
            label=label,
        )

    for radius_m, color, linestyle, alpha, label in [
        (GPS_SIGMA_M, "#6baed6", "-", 0.7, "1 sigma GPS"),
        (2.0 * GPS_SIGMA_M, "#3182bd", "-", 0.7, "2 sigma GPS"),
        (LATE_GATE_M, "#31a354", "--", 0.7, "7 m gate"),
    ]:
        startup_ax.add_patch(
            Circle(
                (origin_truth_xy[0], origin_truth_xy[1]),
                radius_m,
                fill=False,
                color=color,
                linestyle=linestyle,
                linewidth=1.4,
                alpha=alpha,
                zorder=1,
                label=label,
            )
        )

    startup_ax.set_title("Startup geometry at readiness")
    startup_ax.set_xlabel("x [m]")
    startup_ax.set_ylabel("y [m]")
    startup_ax.set_aspect("equal", adjustable="box")
    startup_ax.grid(True, alpha=0.25)
    startup_ax.legend(
        loc="upper center",
        bbox_to_anchor=(0.5, -0.14),
        fontsize=9,
        ncol=2,
        frameon=True,
    )
    fig.colorbar(scatter, ax=startup_ax, fraction=0.046, pad=0.04, label="GPS order")

    history_ax.plot(
        gps_points_used,
        projected_separation,
        color="black",
        linewidth=2.0,
        label="Projected separation",
    )
    history_ax.step(
        gps_points_used,
        required_wheel_supported_travel,
        where="post",
        color="#d62728",
        linewidth=1.8,
        linestyle="--",
        label="Required wheel-supported travel",
    )
    history_ax.plot(
        gps_points_used,
        wheel_supported_travel,
        color="#ff7f0e",
        linewidth=2.0,
        label="Wheel-supported travel",
    )
    history_ax.plot(
        gps_points_used,
        truth_radius,
        color="#1f77b4",
        linewidth=2.0,
        linestyle="--",
        label="Truth radius from start",
    )
    history_ax.plot(
        gps_points_used,
        gps_radius,
        color="#2ca02c",
        linewidth=1.6,
        linestyle=":",
        label="GPS radius from start truth",
    )
    history_ax.axvline(
        metrics["ready_gps_points_used"],
        color="#ff7f0e",
        linewidth=1.5,
        linestyle="--",
        label="Readiness trigger",
    )
    history_ax.axhline(
        2.0 * GPS_SIGMA_M,
        color="#3182bd",
        linewidth=1.2,
        linestyle="-.",
        label="2 sigma GPS scale",
    )
    history_ax.set_title("Startup travel and geometry history")
    history_ax.set_xlabel("GPS points used by startup")
    history_ax.set_ylabel("meters")
    history_ax.grid(True, alpha=0.25)
    history_ax.legend(loc="upper left", fontsize=9)

    speed_ratio_text = format_optional(metrics["speed_ratio"], ".2f")
    startup_note = (
        f"ready at {metrics['startup_duration_s']:.2f} s\n"
        f"truth radius: {metrics['truth_radius_at_ready_m']:.2f} m\n"
        f"projected sep: {metrics['ready_projected_separation_m']:.2f} m\n"
        f"wheel travel: {metrics['ready_wheel_supported_travel_m']:.2f} m\n"
        f"yaw error: {metrics['wrapped_yaw_error_deg']:.1f} deg\n"
        f"speed ratio: {speed_ratio_text}"
    )
    history_ax.text(
        0.98,
        0.03,
        startup_note,
        transform=history_ax.transAxes,
        va="bottom",
        ha="right",
        fontsize=10,
        bbox={
            "boxstyle": "round",
            "facecolor": "white",
            "alpha": 0.9,
            "edgecolor": "#bbbbbb",
        },
    )

    traj_ax.plot(
        run_truth_xy[:, 0],
        run_truth_xy[:, 1],
        color="#1f77b4",
        linewidth=2.0,
        alpha=0.85,
        label="Truth",
    )
    traj_ax.scatter(
        run_gps_xy[:, 0],
        run_gps_xy[:, 1],
        color="black",
        s=18,
        alpha=0.55,
        label="Raw GPS updates",
    )
    traj_ax.plot(
        est_xy[:, 0],
        est_xy[:, 1],
        color="#d62728",
        linewidth=2.0,
        alpha=0.85,
        label="ESKF estimate",
    )
    traj_ax.scatter(
        [est_xy[0, 0]],
        [est_xy[0, 1]],
        color="#ff7f0e",
        s=100,
        marker="D",
        label="First post-startup estimate",
    )
    traj_ax.set_title("Post-startup run in world frame")
    traj_ax.set_xlabel("x [m]")
    traj_ax.set_ylabel("y [m]")
    traj_ax.set_aspect("equal", adjustable="box")
    traj_ax.grid(True, alpha=0.25)
    traj_ax.legend(loc="best")

    update_index = np.arange(len(raw_error_norm))
    err_ax.plot(
        update_index,
        raw_error_norm,
        color="black",
        linewidth=1.8,
        label="Raw GPS error norm",
    )
    err_ax.plot(
        update_index,
        eskf_error_norm,
        color="#d62728",
        linewidth=2.0,
        label="ESKF error norm",
    )
    err_ax.set_title("Post-startup horizontal error")
    err_ax.set_xlabel("Post-startup GPS update index")
    err_ax.set_ylabel("horizontal error [m]")
    err_ax.grid(True, alpha=0.25)
    err_ax.legend(loc="upper left")

    error_note = (
        f"updates: {log_summary['num_updates']}\n"
        f"ESKF beats raw on {log_summary['eskf_beats_raw_count']} / {log_summary['num_updates']}\n"
        f"final raw error: {raw_error_norm[-1]:.2f} m\n"
        f"final ESKF error: {eskf_error_norm[-1]:.2f} m"
    )
    err_ax.text(
        0.98,
        0.97,
        error_note,
        transform=err_ax.transAxes,
        va="top",
        ha="right",
        fontsize=10,
        bbox={
            "boxstyle": "round",
            "facecolor": "white",
            "alpha": 0.9,
            "edgecolor": "#bbbbbb",
        },
    )

    fig.suptitle(f"{scene_id}: startup readiness overview", fontsize=16, y=0.96)
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.95))

    plot_path = READINESS_PLOTS_ROOT / f"{scene_id}_startup_readiness.png"
    fig.savefig(plot_path, dpi=180)
    plt.close(fig)
    return plot_path


def compute_handoff_metrics(
    scene_id: str,
    trace: dict,
    pose_entries: list[dict],
    log_rows: list[dict[str, str]],
    app_result: dict,
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
        "scene_id": scene_id,
        "startup_begin_utime": startup_begin_utime,
        "ready_utime": int(ready_frame["end_utime"]),
        "handoff_utime": handoff_utime,
        "first_runtime_gps_utime": first_runtime_gps_utime,
        "ready_gps_points_used": int(ready_frame["gps_points_used"]),
        "wheel_supported_travel_at_ready_m": float(
            ready_frame["wheel_supported_travel_m"]
        ),
        "raw_gps_rmse_xy_m": float(app_result["raw_gps_rmse_xy"]),
        "eskf_rmse_xy_m": float(app_result["eskf_rmse_xy"]),
        "rmse_ratio": float(app_result["eskf_rmse_xy"] / app_result["raw_gps_rmse_xy"]),
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


def plot_handoff_overview(
    scene_id: str,
    trace: dict,
    pose_entries: list[dict],
    log_rows: list[dict[str, str]],
    metrics: dict,
) -> Path:
    HANDOFF_PLOTS_ROOT.mkdir(parents=True, exist_ok=True)

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
    fig.colorbar(
        startup_scatter,
        ax=startup_ax,
        fraction=0.046,
        pad=0.04,
        label="startup GPS order",
    )
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

    full_ax.set_title("Post-startup geometry in world frame")
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

    fig.suptitle(f"{scene_id}: startup handoff overview", fontsize=16, y=0.98)
    fig.tight_layout(rect=(0.0, 0.02, 1.0, 0.96), h_pad=2.2, w_pad=2.0)

    plot_path = HANDOFF_PLOTS_ROOT / f"{scene_id}_startup_handoff.png"
    fig.savefig(plot_path, dpi=180)
    plt.close(fig)
    return plot_path


def process_scene(scene_id: str) -> dict:
    app_result = run_app_for_scene(scene_id, APP_RUNS_ROOT)
    trace = run_trace_for_scene(scene_id, TRACES_ROOT)
    pose_entries = load_pose_entries(scene_id)

    trace_metrics = compute_trace_overview_metrics(trace, pose_entries)
    trace_plot_path = plot_trace_overview(scene_id, trace, pose_entries, trace_metrics)

    scene_summary = {
        "scene_id": scene_id,
        "app_status": str(app_result["status"]),
        "exit_code": int(app_result["exit_code"]),
        "failure_message": app_result.get("failure_message"),
        "gps_updates": (
            int(app_result["gps_updates"])
            if str(app_result["status"]) == "completed"
            else None
        ),
        "raw_gps_rmse_xy_m": (
            float(app_result["raw_gps_rmse_xy"])
            if str(app_result["status"]) == "completed"
            else None
        ),
        "eskf_rmse_xy_m": (
            float(app_result["eskf_rmse_xy"])
            if str(app_result["status"]) == "completed"
            else None
        ),
        "trace_overview_plot_path": str(trace_plot_path),
        "trace_overview": trace_metrics,
        "readiness_plot_path": None,
        "readiness": None,
        "handoff_plot_path": None,
        "handoff": None,
    }

    if (
        str(app_result["status"]) == "completed"
        and trace["ready_frame_index"] is not None
        and trace["startup_initialization"] is not None
    ):
        log_rows = load_log_rows(Path(app_result["run_dir"]))
        readiness_metrics = compute_readiness_metrics(trace, pose_entries, app_result)
        readiness_plot_path = plot_readiness_overview(
            scene_id, trace, pose_entries, log_rows, readiness_metrics
        )
        handoff_metrics = compute_handoff_metrics(
            scene_id, trace, pose_entries, log_rows, app_result
        )
        handoff_plot_path = plot_handoff_overview(
            scene_id, trace, pose_entries, log_rows, handoff_metrics
        )
        scene_summary["readiness_plot_path"] = str(readiness_plot_path)
        scene_summary["readiness"] = readiness_metrics
        scene_summary["handoff_plot_path"] = str(handoff_plot_path)
        scene_summary["handoff"] = handoff_metrics

    return scene_summary


def summary_markdown(summary: dict) -> str:
    failed_scenes = summary["failed_scenes"]
    readiness_scenes = summary["readiness_plot_count"]
    handoff_scenes = summary["handoff_plot_count"]

    lines = [
        "# Startup Diagnostics",
        "",
        f"- branch: `{summary['branch']}`",
        f"- commit: `{summary['commit']}`",
        f"- GPS seed: `{summary['gps_seed']}`",
        f"- worker count: `{summary['worker_count']}`",
        f"- scenes processed: `{summary['scene_count']}`",
        f"- completed runs: `{summary['completed_count']}`",
        f"- failed runs: `{summary['failed_count']}`",
        f"- trace overview plots: `{summary['trace_overview_plot_count']}`",
        f"- readiness plots: `{readiness_scenes}`",
        f"- handoff plots: `{handoff_scenes}`",
        f"- total elapsed: `{summary['total_elapsed_s']:.2f} s`",
    ]

    if failed_scenes:
        lines.append("- failed scenes: " + ", ".join(failed_scenes))
    else:
        lines.append("- failed scenes: none")

    if summary["median_startup_duration_s"] is not None:
        lines.append(
            f"- median startup duration: `{summary['median_startup_duration_s']:.3f} s`"
        )
        lines.append(
            f"- median ready GPS points: `{summary['median_ready_gps_points_used']:.1f}`"
        )
        lines.append(
            f"- median handoff speed error: `{summary['median_handoff_speed_error_mps']:.6f} m/s`"
        )
        lines.append(
            f"- median absolute handoff heading error: "
            f"`{summary['median_abs_handoff_heading_error_deg']:.6f} deg`"
        )

    if summary["worst_abs_handoff_heading_error_scenes"]:
        lines.append("")
        lines.append("## Largest absolute handoff heading errors")
        lines.append("")
        for entry in summary["worst_abs_handoff_heading_error_scenes"]:
            lines.append(
                f"- {entry['scene_id']}: `{entry['handoff_heading_error_deg']:+.6f} deg`"
            )

    return "\n".join(lines) + "\n"


def build_summary(scene_summaries: list[dict], total_elapsed_s: float) -> dict:
    completed_scenes = [
        scene_summary
        for scene_summary in scene_summaries
        if scene_summary["app_status"] == "completed"
    ]
    failed_scenes = [
        scene_summary["scene_id"]
        for scene_summary in scene_summaries
        if scene_summary["app_status"] == "failed"
    ]
    readiness_summaries = [
        scene_summary["readiness"]
        for scene_summary in scene_summaries
        if scene_summary["readiness"] is not None
    ]
    handoff_summaries = [
        scene_summary["handoff"]
        for scene_summary in scene_summaries
        if scene_summary["handoff"] is not None
    ]

    summary = {
        "branch": current_branch_name(),
        "commit": current_commit_short_sha(),
        "run_label": RUN_LABEL,
        "gps_seed": DEFAULT_GPS_SEED,
        "worker_count": WORKER_COUNT,
        "scene_count": len(scene_summaries),
        "completed_count": len(completed_scenes),
        "failed_count": len(failed_scenes),
        "failed_scenes": failed_scenes,
        "trace_overview_plot_count": len(scene_summaries),
        "readiness_plot_count": len(readiness_summaries),
        "handoff_plot_count": len(handoff_summaries),
        "total_elapsed_s": total_elapsed_s,
        "median_startup_duration_s": (
            statistics.median(
                readiness_summary["startup_duration_s"]
                for readiness_summary in readiness_summaries
            )
            if readiness_summaries
            else None
        ),
        "median_ready_gps_points_used": (
            statistics.median(
                readiness_summary["ready_gps_points_used"]
                for readiness_summary in readiness_summaries
            )
            if readiness_summaries
            else None
        ),
        "median_handoff_speed_error_mps": (
            statistics.median(
                handoff_summary["handoff_speed_error_mps"]
                for handoff_summary in handoff_summaries
            )
            if handoff_summaries
            else None
        ),
        "median_abs_handoff_heading_error_deg": (
            statistics.median(
                abs(handoff_summary["handoff_heading_error_deg"])
                for handoff_summary in handoff_summaries
            )
            if handoff_summaries
            else None
        ),
        "worst_abs_handoff_heading_error_scenes": sorted(
            [
                {
                    "scene_id": handoff_summary["scene_id"],
                    "handoff_heading_error_deg": handoff_summary[
                        "handoff_heading_error_deg"
                    ],
                }
                for handoff_summary in handoff_summaries
            ],
            key=lambda entry: abs(float(entry["handoff_heading_error_deg"])),
            reverse=True,
        )[:10],
        "scenes": scene_summaries,
    }
    return summary


def main() -> None:
    ensure_binaries_exist(require_trace=True)

    scene_ids = list_scene_ids()
    make_clean_dir(RESULTS_ROOT)
    make_clean_dir(SCRATCH_ROOT)

    start_time = time.perf_counter()
    scene_summaries = []
    with concurrent.futures.ProcessPoolExecutor(max_workers=WORKER_COUNT) as executor:
        futures = {
            executor.submit(process_scene, scene_id): scene_id for scene_id in scene_ids
        }
        for future in concurrent.futures.as_completed(futures):
            scene_summary = future.result()
            scene_summaries.append(scene_summary)
            print(f"Wrote startup diagnostics for {scene_summary['scene_id']}")

    total_elapsed_s = time.perf_counter() - start_time
    scene_summaries.sort(key=lambda scene_summary: scene_summary["scene_id"])

    summary = build_summary(scene_summaries, total_elapsed_s)
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
