"""
generate_plots.py

Generates plots of the numerical differences between two runs: per-cluster
bar charts, distribution histograms, and a scatter of centroid vs.
orientation error.
"""
from __future__ import annotations

from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd

_COLOR_PRIMARY = "#3C3489"
_COLOR_SECONDARY = "#993C1D"
_COLOR_TERTIARY = "#B7534A"
_COLOR_NEUTRAL = "#5F5E5A"


def _bar_plot(df: pd.DataFrame, column: str, title: str, ylabel: str, path: Path, color: str) -> None:
    fig, ax = plt.subplots(figsize=(max(6.0, 0.4 * len(df)), 4.0))
    labels = df["context_a"].astype(str) if "context_a" in df.columns else df.index.astype(str)
    ax.bar(labels, df[column], color=color)
    ax.set_title(title)
    ax.set_xlabel("Cluster (baseline context)")
    ax.set_ylabel(ylabel)
    ax.tick_params(axis="x", rotation=45)
    ax.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def _hist_plot(df: pd.DataFrame, column: str, title: str, xlabel: str, path: Path, color: str) -> None:
    fig, ax = plt.subplots(figsize=(6.0, 4.0))
    data = df[column].dropna()
    bins = min(20, max(3, len(data) // 2)) if len(data) > 0 else 1
    ax.hist(data, bins=bins, color=color, edgecolor="white")
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel("Count")
    ax.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def generate_plots(result: dict, output_dir: Path) -> None:
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    df = result["per_cluster"]
    if df.empty:
        print("No matched clusters; skipping plots.")
        return

    _bar_plot(df, "centroid_error", "Centroid error per cluster", "Euclidean error (m)",
              output_dir / "centroid_error_per_cluster.png", _COLOR_PRIMARY)
    _bar_plot(df, "bbox_center_error", "Bounding box center error per cluster", "Euclidean error (m)",
              output_dir / "bbox_center_error_per_cluster.png", _COLOR_SECONDARY)
    _bar_plot(df, "orientation_error_deg", "Orientation error per cluster", "Angle error (degrees)",
              output_dir / "orientation_error_per_cluster.png", _COLOR_TERTIARY)
    _bar_plot(df, "half_extent_error_norm", "Bounding box size error per cluster", "||half-extent error|| (m)",
              output_dir / "bbox_size_error_per_cluster.png", _COLOR_NEUTRAL)
    _bar_plot(df, "eigenvalue_error_norm", "PCA eigenvalue error per cluster", "||eigenvalue error||",
              output_dir / "eigenvalue_error_per_cluster.png", _COLOR_PRIMARY)

    _hist_plot(df, "centroid_error", "Distribution of centroid error", "Euclidean error (m)",
               output_dir / "centroid_error_histogram.png", _COLOR_PRIMARY)
    _hist_plot(df, "orientation_error_deg", "Distribution of orientation error", "Angle error (degrees)",
               output_dir / "orientation_error_histogram.png", _COLOR_TERTIARY)

    fig, ax = plt.subplots(figsize=(6.0, 4.5))
    ax.scatter(df["centroid_error"], df["orientation_error_deg"], color=_COLOR_PRIMARY)
    for _, r in df.iterrows():
        ax.annotate(str(r["context_a"]), (r["centroid_error"], r["orientation_error_deg"]),
                    fontsize=7, alpha=0.7, xytext=(3, 3), textcoords="offset points")
    ax.set_xlabel("Centroid error (m)")
    ax.set_ylabel("Orientation error (degrees)")
    ax.set_title("Centroid error vs. orientation error, per cluster")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_dir / "centroid_vs_orientation_error.png", dpi=150)
    plt.close(fig)

    # Stacked summary: mean/max for the four headline metrics, side by side.
    headline = ["centroid_error", "bbox_center_error", "half_extent_error_norm", "orientation_error_deg"]
    means = [df[c].mean() for c in headline]
    maxes = [df[c].max() for c in headline]
    fig, ax = plt.subplots(figsize=(7.0, 4.5))
    x = range(len(headline))
    width = 0.35
    ax.bar([i - width / 2 for i in x], means, width, label="mean", color=_COLOR_PRIMARY)
    ax.bar([i + width / 2 for i in x], maxes, width, label="max", color=_COLOR_TERTIARY)
    ax.set_xticks(list(x))
    ax.set_xticklabels(["centroid\nerror (m)", "bbox center\nerror (m)",
                         "bbox size\nerror (m)", "orientation\nerror (deg)"])
    ax.set_title("Headline metrics: mean vs. max across matched clusters")
    ax.legend()
    ax.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_dir / "headline_metrics_summary.png", dpi=150)
    plt.close(fig)

    print(f"Wrote plots to {output_dir}")
