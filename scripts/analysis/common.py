"""
common.py

Shared utilities for comparing two perception_sim runs from their CSV
instrumentation (data/csv/*.csv, written by perception_sim's CsvLogger).
"""
from __future__ import annotations

import re
from pathlib import Path

import numpy as np
import pandas as pd

# Matches context strings that identify a whole cluster/object, e.g.
# "cluster_2" - excludes per-point ("cluster_2:point_14") and per-corner
# ("cluster_2:corner_5") rows, which are logged with a colon-suffixed context.
CLUSTER_CONTEXT_RE = re.compile(r"^cluster_(\d+)$")


def load_csv(csv_dir: Path, name: str) -> pd.DataFrame:
    """Loads one of perception_sim's CSV instrumentation files by category
    name (e.g. "centroids", "bounding_boxes"), without filtering by run."""
    path = Path(csv_dir) / f"{name}.csv"
    if not path.exists():
        raise FileNotFoundError(
            f"Expected CSV not found: {path}\n"
            f"Make sure perception_sim_app has been run at least once with "
            f"CSV instrumentation enabled (see CsvLogger::beginRun)."
        )
    return pd.read_csv(path)


def filter_run(df: pd.DataFrame, run_id: str) -> pd.DataFrame:
    """Returns only the rows belonging to `run_id`."""
    out = df[df["run_id"] == run_id].copy()
    if out.empty:
        available = sorted(df["run_id"].unique()) if "run_id" in df.columns else []
        raise ValueError(
            f"No rows found for run_id={run_id!r}. "
            f"Available run_ids in this file: {available}"
        )
    return out


def cluster_rows_only(df: pd.DataFrame) -> pd.DataFrame:
    """Keeps only rows whose context is exactly 'cluster_<N>' (i.e. drops
    per-point / per-corner instrumentation rows), for CSVs where such rows
    can appear (matrix_multiplications, or bounding_boxes' corner rows are
    not written this way, but this helper is safe to apply broadly)."""
    if "context" not in df.columns:
        return df
    mask = df["context"].astype(str).str.match(CLUSTER_CONTEXT_RE)
    return df[mask].copy()


def with_cluster_index(df: pd.DataFrame) -> pd.DataFrame:
    """Adds an integer `_cluster_idx` column parsed from a 'cluster_<N>'
    context, and sorts/reindexes by it for stable, predictable ordering."""
    df = df.copy()
    df["_cluster_idx"] = df["context"].astype(str).str.extract(r"cluster_(\d+)").astype(int)
    df = df.sort_values("_cluster_idx").reset_index(drop=True)
    return df


def axes_matrix(row: pd.Series) -> np.ndarray:
    """Builds the 3x3 orientation matrix (columns = box axes) from a
    bounding_boxes.csv row."""
    return np.array([
        [row.axis0_x, row.axis1_x, row.axis2_x],
        [row.axis0_y, row.axis1_y, row.axis2_y],
        [row.axis0_z, row.axis1_z, row.axis2_z],
    ], dtype=float)


def eigenvectors_matrix(row: pd.Series) -> np.ndarray:
    """Builds the 3x3 eigenvector matrix (columns = eigenvectors, ordered to
    match lambda0..lambda2) from a pca_eigenvectors.csv row."""
    return np.array([
        [row.v0_x, row.v1_x, row.v2_x],
        [row.v0_y, row.v1_y, row.v2_y],
        [row.v0_z, row.v1_z, row.v2_z],
    ], dtype=float)


def align_axis_signs(R_ref: np.ndarray, R: np.ndarray) -> np.ndarray:
    """PCA eigenvectors (and the box axes derived from them) are only
    defined up to sign - an eigensolver is free to return either +v or -v
    for the same axis. Before computing any orientation/rotation error,
    flip each column of R so its dot product with the corresponding column
    of R_ref is non-negative. Without this step, a purely cosmetic sign
    flip would look like a ~180 degree orientation error."""
    R_aligned = R.copy()
    for col in range(R.shape[1]):
        if np.dot(R_ref[:, col], R[:, col]) < 0:
            R_aligned[:, col] *= -1
    return R_aligned


def rotation_angle_deg(R_ref: np.ndarray, R: np.ndarray) -> float:
    """Geodesic angle (in degrees) between two orthonormal 3x3 frames, after
    sign-aligning R to R_ref column-wise (see align_axis_signs)."""
    R_aligned = align_axis_signs(R_ref, R)
    R_rel = R_ref.T @ R_aligned
    cos_angle = (np.trace(R_rel) - 1.0) / 2.0
    cos_angle = float(np.clip(cos_angle, -1.0, 1.0))
    return float(np.degrees(np.arccos(cos_angle)))


def match_clusters(centroids_a: np.ndarray, centroids_b: np.ndarray):
    """Matches clusters between two runs by nearest centroid, using the
    Hungarian algorithm (optimal assignment) on pairwise Euclidean distance.

    This is necessary because a modified Eigen could, in principle, shift
    cluster membership enough to change cluster count or ordering (see
    Path 2 in the dependency analysis: PCL's internal Eigen-backed point
    storage feeds voxel filtering and clustering). Matching by index alone
    would silently misattribute differences when counts/order don't line up.

    Returns:
        matches: list of (idx_a, idx_b, distance) tuples
        unmatched_a: indices into centroids_a with no counterpart in b
        unmatched_b: indices into centroids_b with no counterpart in a
    """
    from scipy.optimize import linear_sum_assignment

    n_a, n_b = len(centroids_a), len(centroids_b)
    if n_a == 0 or n_b == 0:
        return [], list(range(n_a)), list(range(n_b))

    cost = np.linalg.norm(centroids_a[:, None, :] - centroids_b[None, :, :], axis=2)
    row_ind, col_ind = linear_sum_assignment(cost)
    matches = [(int(r), int(c), float(cost[r, c])) for r, c in zip(row_ind, col_ind)]

    matched_a = {r for r, _, _ in matches}
    matched_b = {c for _, c, _ in matches}
    unmatched_a = [i for i in range(n_a) if i not in matched_a]
    unmatched_b = [i for i in range(n_b) if i not in matched_b]
    return matches, unmatched_a, unmatched_b
