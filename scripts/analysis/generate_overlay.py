"""
generate_overlay.py

Renders a top-down (x/y) overlay of both runs' bounding-box footprints and
centroids, with displacement lines connecting matched clusters, so numerical
drift between a stock-Eigen run and a modified-Eigen run is visible at a
glance - independent of perception_sim's own OpenCV rendering (which only
ever shows one run at a time).
"""
from __future__ import annotations

from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.lines import Line2D
import numpy as np

import common

_COLOR_A = "#3C3489"
_COLOR_B = "#D9822B"
_COLOR_LINK = "#999999"


def _footprint_corners(center: np.ndarray, axes: np.ndarray, half_extents: np.ndarray):
    """Returns the 4 ground-plane (x, y) corners of a box's footprint, using
    the first two PCA axes projected onto the x/y plane. This is an
    approximation for a 2D overlay of a full 3D oriented box - it shows the
    box's footprint as if viewed from directly above."""
    cx, cy = center[0], center[1]
    ax0 = axes[:2, 0] * half_extents[0]
    ax1 = axes[:2, 1] * half_extents[1]
    corners = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            corners.append((cx + sx * ax0[0] + sy * ax1[0], cy + sx * ax0[1] + sy * ax1[1]))
    # Reorder the 4 signed combinations into a non-self-intersecting polygon.
    return [corners[0], corners[1], corners[3], corners[2]]


def _draw_boxes(ax, bounding_boxes_df, color: str, linestyle: str) -> None:
    for _, row in bounding_boxes_df.iterrows():
        center = np.array([row.center_x, row.center_y, row.center_z])
        axes_m = common.axes_matrix(row)
        half = np.array([row.half_extent_x, row.half_extent_y, row.half_extent_z])
        poly = _footprint_corners(center, axes_m, half)
        ax.add_patch(patches.Polygon(poly, closed=True, fill=False,
                                       edgecolor=color, linewidth=2, linestyle=linestyle))


def generate_overlay(result: dict, output_dir: Path) -> None:
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    a, b = result["run_a"], result["run_b"]
    df = result["per_cluster"]

    fig, ax = plt.subplots(figsize=(8.0, 8.0))

    _draw_boxes(ax, a.bounding_boxes, _COLOR_A, "-")
    _draw_boxes(ax, b.bounding_boxes, _COLOR_B, "--")

    if not a.cluster_centers.empty:
        ax.scatter(a.cluster_centers.x, a.cluster_centers.y, color=_COLOR_A, marker="o", s=25, zorder=3)
    if not b.cluster_centers.empty:
        ax.scatter(b.cluster_centers.x, b.cluster_centers.y, color=_COLOR_B, marker="x", s=35, zorder=3)

    # Displacement line between each matched pair of centroids.
    for _, r in df.iterrows():
        ca = a.cluster_centers[a.cluster_centers.context == r.context_a].iloc[0]
        cb = b.cluster_centers[b.cluster_centers.context == r.context_b].iloc[0]
        ax.plot([ca.x, cb.x], [ca.y, cb.y], color=_COLOR_LINK, linewidth=1, linestyle=":", zorder=2)

    # Mark unmatched (extra/missing) clusters distinctly.
    for rec in result["unmatched_a"]:
        ax.scatter([rec["x"]], [rec["y"]], color=_COLOR_A, marker="s", s=80,
                   facecolors="none", linewidths=2, zorder=4)
    for rec in result["unmatched_b"]:
        ax.scatter([rec["x"]], [rec["y"]], color=_COLOR_B, marker="s", s=80,
                   facecolors="none", linewidths=2, zorder=4)

    legend_elems = [
        Line2D([0], [0], color=_COLOR_A, lw=2, label=f"Run A: {a.run_id}"),
        Line2D([0], [0], color=_COLOR_B, lw=2, linestyle="--", label=f"Run B: {b.run_id}"),
        Line2D([0], [0], color=_COLOR_LINK, lw=1, linestyle=":", label="Matched centroid displacement"),
        Line2D([0], [0], marker="s", color="w", markeredgecolor="black", markerfacecolor="none",
               markersize=10, label="Unmatched cluster (count mismatch)"),
    ]
    ax.legend(handles=legend_elems, loc="upper right", fontsize=8)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Top-down overlay: bounding box footprints and centroids")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    out_path = output_dir / "overlay_topdown.png"
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"Wrote overlay to {out_path}")
