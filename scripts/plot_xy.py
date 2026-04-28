#!/usr/bin/env python3

from __future__ import annotations

import csv
import math
import os
import sys
import tempfile
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "matplotlib"))
os.environ.setdefault("XDG_CACHE_HOME", str(Path(tempfile.gettempdir()) / "xdg-cache"))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


CSV_PATH = Path("/home/dfw/dev/imu-gps-eskf/output/eskf_sim_log.csv")
OUTPUT_PATH = Path("/home/dfw/dev/imu-gps-eskf/assets/xy_trajectory.png")

REQUIRED_COLUMNS = (
    "est_x",
    "est_y",
    "gps_x",
    "gps_y",
    "true_x",
    "true_y",
)


def read_log(path: Path) -> dict[str, list[float]]:
    if not path.exists():
        raise FileNotFoundError(f"Input CSV not found: {path}")

    series = {name: [] for name in REQUIRED_COLUMNS}

    with path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        missing = [name for name in REQUIRED_COLUMNS if name not in (reader.fieldnames or [])]
        if missing:
            missing_txt = ", ".join(missing)
            raise ValueError(f"CSV missing required columns: {missing_txt}")

        for row in reader:
            for name in REQUIRED_COLUMNS:
                series[name].append(float(row[name]))

    if not series["est_x"]:
        raise ValueError(f"CSV has no data rows: {path}")

    return series


def rmse_xy(ax: list[float], ay: list[float], bx: list[float], by: list[float]) -> float:
    if not (len(ax) == len(ay) == len(bx) == len(by)):
        raise ValueError("RMSE input lengths do not match")

    n = len(ax)
    if n == 0:
        raise ValueError("RMSE requires at least one sample")

    sum_sq = 0.0
    for i in range(n):
        dx = ax[i] - bx[i]
        dy = ay[i] - by[i]
        sum_sq += dx * dx + dy * dy
    return math.sqrt(sum_sq / n)


INK = "#111827"
FOG = "#F8FAFC"
ESTIMATION_BLUE = "#60A5FA"
SENSOR_TEAL = "#0F766E"
SIGNAL_AMBER = "#F59E0B"
GRID = "#CBD5E1"


def make_plot(series: dict[str, list[float]], output_path: Path) -> tuple[float, float]:
    est_x = series["est_x"]
    est_y = series["est_y"]
    gps_x = series["gps_x"]
    gps_y = series["gps_y"]
    true_x = series["true_x"]
    true_y = series["true_y"]

    rmse_gps_xy = rmse_xy(gps_x, gps_y, true_x, true_y)
    rmse_eskf_xy = rmse_xy(est_x, est_y, true_x, true_y)

    fig, ax = plt.subplots(figsize=(8, 8))
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")

    # Plot GPS first so the two trajectories sit on top of the cloud.
    ax.scatter(
        gps_x,
        gps_y,
        label="GPS",
        color=SENSOR_TEAL,
        s=18,
        alpha=0.55,
        edgecolors=FOG,
        linewidths=0.35,
        zorder=1,
    )

    # Plot ESKF as the primary result.
    ax.plot(
        est_x,
        est_y,
        label="ESKF",
        color=ESTIMATION_BLUE,
        linewidth=2.8,
        zorder=3,
    )

    # Plot truth on top as a dashed reference so it stays distinguishable
    # even where it closely overlaps the estimate.
    ax.plot(
        true_x,
        true_y,
        label="Truth",
        color=INK,
        linewidth=2.1,
        linestyle=(0, (6, 3)),
        zorder=4,
    )

    start_marker = ax.scatter(
        true_x[0],
        true_y[0],
        marker="o",
        s=70,
        color=SIGNAL_AMBER,
        edgecolors=INK,
        linewidths=0.8,
        zorder=5,
    )
    finish_marker = ax.scatter(
        true_x[-1],
        true_y[-1],
        marker="X",
        s=95,
        color=SIGNAL_AMBER,
        edgecolors=INK,
        linewidths=0.8,
        zorder=5,
    )

    start_annotation = ax.annotate(
        "Start",
        xy=(true_x[0], true_y[0]),
        xytext=(0, 8),
        textcoords="offset points",
        fontsize=9,
        color=INK,
        ha="left",
        va="bottom",
    )
    finish_annotation = ax.annotate(
        "Finish",
        xy=(true_x[-1], true_y[-1]),
        xytext=(0, 0),
        textcoords="offset points",
        fontsize=9,
        color=INK,
        ha="left",
        va="bottom",
    )

    rmse_text = ax.text(
        0.98,
        0.02,
        f"Raw GPS RMSE (xy): {rmse_gps_xy:.3f} m\n"
        f"ESKF RMSE (xy): {rmse_eskf_xy:.3f} m",
        transform=ax.transAxes,
        fontsize=10,
        color=INK,
        ha="right",
        va="bottom",
        bbox={
            "boxstyle": "round,pad=0.35",
            "facecolor": FOG,
            "alpha": 0.96,
            "edgecolor": GRID,
            "linewidth": 1.0,
        },
    )

    ax.set_xlabel("x [m]", color=INK)
    ax.set_ylabel("y [m]", color=INK)
    ax.set_title("XY trajectory overlay", color=INK)

    ax.tick_params(colors=INK)
    ax.grid(True, color=GRID, alpha=0.45, linewidth=0.8)
    ax.set_aspect("equal", adjustable="box")
    ax.margins(x=0.03, y=0.08)
    ymin, ymax = ax.get_ylim()
    ax.set_ylim(ymin, (5.0 / 3.0) * ymax)

    legend = ax.legend(loc="upper left", frameon=True)
    legend.get_frame().set_facecolor(FOG)
    legend.get_frame().set_edgecolor(GRID)
    legend.get_frame().set_alpha(0.96)

    fig.tight_layout()

    def marker_bbox_px(marker_collection):
        path = marker_collection.get_paths()[0]
        marker_to_px = marker_collection.get_transforms()[0]
        marker_path_px = path.transformed(
            matplotlib.transforms.Affine2D(marker_to_px))
        path_bbox = marker_path_px.get_extents()
        x_data, y_data = marker_collection.get_offsets()[0]
        x_px, y_px = ax.transData.transform((float(x_data), float(y_data)))
        return matplotlib.transforms.Bbox.from_extents(
            path_bbox.x0 + x_px,
            path_bbox.y0 + y_px,
            path_bbox.x1 + x_px,
            path_bbox.y1 + y_px,
        )

    fig.canvas.draw()
    renderer = fig.canvas.get_renderer()
    axes_bbox = ax.get_window_extent(renderer=renderer)
    legend_bbox = legend.get_window_extent(renderer=renderer)
    rmse_bbox = rmse_text.get_bbox_patch().get_window_extent(renderer=renderer)
    legend_top_gap_px = axes_bbox.y1 - legend_bbox.y1
    target_rmse_y0_px = axes_bbox.y0 + legend_top_gap_px
    rmse_dy_px = target_rmse_y0_px - rmse_bbox.y0
    rmse_x_axes, rmse_y_axes = rmse_text.get_position()
    rmse_anchor_x_px, rmse_anchor_y_px = ax.transAxes.transform(
        (rmse_x_axes, rmse_y_axes))
    _, target_rmse_y_axes = ax.transAxes.inverted().transform(
        (rmse_anchor_x_px, rmse_anchor_y_px + rmse_dy_px))
    rmse_text.set_position((rmse_x_axes, target_rmse_y_axes))
    fig.canvas.draw()
    renderer = fig.canvas.get_renderer()
    axes_bbox = ax.get_window_extent(renderer=renderer)
    start_bbox = start_annotation.get_window_extent(renderer=renderer)
    target_start_x0_px = axes_bbox.x0 + 3.0
    dx_px = target_start_x0_px - start_bbox.x0
    start_x_pt, start_y_pt = start_annotation.get_position()
    start_annotation.set_position(
        (start_x_pt + dx_px * 72.0 / fig.dpi, start_y_pt))

    finish_bbox = finish_annotation.get_window_extent(renderer=renderer)
    finish_x_px, finish_y_px = ax.transData.transform((true_x[-1], true_y[-1]))
    finish_font = finish_annotation.get_fontproperties()
    finish_prefix_width_px = renderer.get_text_width_height_descent(
        "Fi", finish_font, ismath=False)[0]
    finish_n_width_px = renderer.get_text_width_height_descent(
        "n", finish_font, ismath=False)[0]
    finish_n_center_from_left_px = finish_prefix_width_px + 0.5 * finish_n_width_px
    target_finish_x0_px = finish_x_px - finish_n_center_from_left_px
    target_finish_y0_px = finish_y_px + 20.0
    finish_dx_px = target_finish_x0_px - finish_bbox.x0
    finish_dy_px = target_finish_y0_px - finish_bbox.y0
    finish_x_pt, finish_y_pt = finish_annotation.get_position()
    finish_annotation.set_position(
        (
            finish_x_pt + finish_dx_px * 72.0 / fig.dpi,
            finish_y_pt + finish_dy_px * 72.0 / fig.dpi,
        ))
    fig.canvas.draw()
    renderer = fig.canvas.get_renderer()

    finish_bbox = finish_annotation.get_window_extent(renderer=renderer)
    finish_marker_bbox = marker_bbox_px(finish_marker)
    start_bbox = start_annotation.get_window_extent(renderer=renderer)
    start_marker_bbox = marker_bbox_px(start_marker)
    finish_text_gap_px = finish_bbox.y0 - finish_marker_bbox.y1
    target_start_y0_px = start_marker_bbox.y1 + finish_text_gap_px
    start_dy_px = target_start_y0_px - start_bbox.y0
    start_x_pt, start_y_pt = start_annotation.get_position()
    start_annotation.set_position(
        (start_x_pt, start_y_pt + start_dy_px * 72.0 / fig.dpi))
    fig.canvas.draw()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=200, bbox_inches="tight")
    plt.close(fig)

    return rmse_gps_xy, rmse_eskf_xy


def main() -> int:
    try:
        series = read_log(CSV_PATH)
        rmse_gps_xy, rmse_eskf_xy = make_plot(series, OUTPUT_PATH)
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    print(f"Wrote plot: {OUTPUT_PATH}")
    print(f"Raw GPS RMSE (xy): {rmse_gps_xy:.6f} m")
    print(f"ESKF RMSE (xy): {rmse_eskf_xy:.6f} m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
