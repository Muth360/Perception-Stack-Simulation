"""
compute_metrics.py

Loads the CSV instrumentation for two runs and computes:
  - centroid error
  - bounding box error (center, half-extent, volume)
  - orientation error
  - cluster count differences
  - covariance / eigenvalue error (bonus numerical-drift signal)

for every matched pair of clusters between the two runs.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np
import pandas as pd

import common


class RunData:
    """All per-cluster instrumentation for a single run_id, loaded from
    data/csv/ and filtered/sorted for easy indexed access."""

    CATEGORIES = [
        "cluster_centers",
        "centroids",
        "covariance_matrices",
        "pca_eigenvalues",
        "pca_eigenvectors",
        "bounding_boxes",
    ]

    def __init__(self, csv_dir: Path, run_id: str):
        self.run_id = run_id
        self.csv_dir = Path(csv_dir)

        for category in self.CATEGORIES:
            df = common.load_csv(self.csv_dir, category)
            df = common.filter_run(df, run_id)
            df = common.cluster_rows_only(df)
            df = common.with_cluster_index(df)
            setattr(self, category, df)

    @property
    def num_clusters(self) -> int:
        return len(self.cluster_centers)


def _half_extents(row: pd.Series) -> np.ndarray:
    return np.array([row.half_extent_x, row.half_extent_y, row.half_extent_z], dtype=float)


def _center(row: pd.Series, prefix: str = "") -> np.ndarray:
    if prefix:
        return np.array([getattr(row, f"{prefix}_x"), getattr(row, f"{prefix}_y"), getattr(row, f"{prefix}_z")])
    return np.array([row.x, row.y, row.z], dtype=float)


def compare_runs(csv_dir: Path, run_id_a: str, run_id_b: str) -> dict:
    """Compares run_id_a (baseline, e.g. stock Eigen) against run_id_b
    (comparison, e.g. modified Eigen). Returns a dict with:

      per_cluster   - DataFrame, one row per matched cluster pair, all metrics
      unmatched_a   - list of dicts: clusters only present in run_a
      unmatched_b   - list of dicts: clusters only present in run_b
      aggregate     - dict of mean/max/std per metric across matched clusters
      cluster_count - dict summarizing counts and the matching outcome
      run_a, run_b  - the underlying RunData objects (used by plotting/overlay)
    """
    a = RunData(csv_dir, run_id_a)
    b = RunData(csv_dir, run_id_b)

    centroids_a = a.cluster_centers[["x", "y", "z"]].to_numpy(dtype=float)
    centroids_b = b.cluster_centers[["x", "y", "z"]].to_numpy(dtype=float)

    matches, unmatched_a_idx, unmatched_b_idx = common.match_clusters(centroids_a, centroids_b)

    rows = []
    for idx_a, idx_b, match_dist in matches:
        ca = a.cluster_centers.iloc[idx_a]
        cb = b.cluster_centers.iloc[idx_b]
        ba = a.bounding_boxes.iloc[idx_a]
        bb = b.bounding_boxes.iloc[idx_b]

        # --- Centroid error ---------------------------------------------
        centroid_a = _center(ca)
        centroid_b = _center(cb)
        centroid_error = float(np.linalg.norm(centroid_a - centroid_b))
        centroid_error_xyz = centroid_b - centroid_a

        # --- Bounding box error ------------------------------------------
        bbox_center_a = _center(ba, prefix="center")
        bbox_center_b = _center(bb, prefix="center")
        bbox_center_error = float(np.linalg.norm(bbox_center_a - bbox_center_b))

        half_a = _half_extents(ba)
        half_b = _half_extents(bb)
        half_extent_error_xyz = half_b - half_a
        half_extent_error_norm = float(np.linalg.norm(half_extent_error_xyz))

        volume_a = float(np.prod(2.0 * half_a))
        volume_b = float(np.prod(2.0 * half_b))
        volume_error = volume_b - volume_a
        volume_error_pct = (volume_error / volume_a * 100.0) if volume_a > 1e-12 else float("nan")

        # --- Orientation error ---------------------------------------------
        R_a = common.axes_matrix(ba)
        R_b = common.axes_matrix(bb)
        orientation_error_deg = common.rotation_angle_deg(R_a, R_b)

        # --- Bonus: covariance / eigenvalue drift (root numerical signal) --
        cov_a = a.covariance_matrices.iloc[idx_a]
        cov_b = b.covariance_matrices.iloc[idx_b]
        cov_cols = ["c00", "c01", "c02", "c11", "c12", "c22"]
        cov_diff = np.array([cov_b[c] - cov_a[c] for c in cov_cols], dtype=float)
        covariance_frobenius_error = float(np.linalg.norm(cov_diff))

        eig_a = a.pca_eigenvalues.iloc[idx_a]
        eig_b = b.pca_eigenvalues.iloc[idx_b]
        eigenvalue_error_norm = float(np.linalg.norm([
            eig_b.lambda0 - eig_a.lambda0,
            eig_b.lambda1 - eig_a.lambda1,
            eig_b.lambda2 - eig_a.lambda2,
        ]))

        rows.append({
            "context_a": ca.context,
            "context_b": cb.context,
            "match_centroid_distance": match_dist,
            "num_points_a": int(ca.num_points),
            "num_points_b": int(cb.num_points),
            "centroid_error": centroid_error,
            "centroid_error_x": centroid_error_xyz[0],
            "centroid_error_y": centroid_error_xyz[1],
            "centroid_error_z": centroid_error_xyz[2],
            "bbox_center_error": bbox_center_error,
            "half_extent_error_x": half_extent_error_xyz[0],
            "half_extent_error_y": half_extent_error_xyz[1],
            "half_extent_error_z": half_extent_error_xyz[2],
            "half_extent_error_norm": half_extent_error_norm,
            "volume_a": volume_a,
            "volume_b": volume_b,
            "volume_error": volume_error,
            "volume_error_pct": volume_error_pct,
            "orientation_error_deg": orientation_error_deg,
            "covariance_frobenius_error": covariance_frobenius_error,
            "eigenvalue_error_norm": eigenvalue_error_norm,
        })

    per_cluster = pd.DataFrame(rows)

    unmatched_a = a.cluster_centers.iloc[unmatched_a_idx][["context", "num_points", "x", "y", "z"]].to_dict("records")
    unmatched_b = b.cluster_centers.iloc[unmatched_b_idx][["context", "num_points", "x", "y", "z"]].to_dict("records")

    aggregate = {}
    metric_cols = [
        "centroid_error", "bbox_center_error", "half_extent_error_norm",
        "orientation_error_deg", "covariance_frobenius_error",
        "eigenvalue_error_norm", "volume_error_pct",
    ]
    if not per_cluster.empty:
        for col in metric_cols:
            aggregate[f"{col}_mean"] = float(per_cluster[col].mean())
            aggregate[f"{col}_max"] = float(per_cluster[col].max())
            aggregate[f"{col}_std"] = float(per_cluster[col].std(ddof=0))

    cluster_count = {
        "run_a": run_id_a,
        "run_b": run_id_b,
        "num_clusters_a": a.num_clusters,
        "num_clusters_b": b.num_clusters,
        "cluster_count_diff": b.num_clusters - a.num_clusters,
        "num_matched": len(matches),
        "num_unmatched_a": len(unmatched_a_idx),
        "num_unmatched_b": len(unmatched_b_idx),
    }

    return {
        "per_cluster": per_cluster,
        "unmatched_a": unmatched_a,
        "unmatched_b": unmatched_b,
        "aggregate": aggregate,
        "cluster_count": cluster_count,
        "run_a": a,
        "run_b": b,
    }
