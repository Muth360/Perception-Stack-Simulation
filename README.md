# perception_sim

A lightweight, standalone simulation of a simplified autonomous-vehicle
perception stack, built for **defensive supply-chain-security research**:
studying how a modified `Eigen` could propagate through `PCL` and `OpenCV`
into perception outputs (centroids, oriented bounding boxes, and the final
visualization), without needing to build the full Autoware Universe stack.

```
Point Cloud
     ↓
PCL Filtering            (filtering.hpp/.cpp — passthrough + voxel grid)
     ↓
Euclidean Clustering     (clustering.hpp/.cpp — PCL KdTree + cluster extraction)
     ↓
Eigen Matrix Operations  (eigen_ops.hpp/.cpp, pca_analysis.*, bounding_box.*)
     ↓
OpenCV Visualization     (visualization.hpp/.cpp)
     ↓
Detected Objects
```

## Why this design

This project's core research question is: *if Eigen's numerics change,
how does that ripple through the rest of the stack?* To make that easy to
observe and reason about:

- **Every direct use of Eigen types/algorithms is isolated in
  `include/perception_sim/eigen_ops.hpp` and `src/eigen_ops.cpp`.** No other
  header or source file in the project includes an Eigen header. Downstream
  modules (`pca_analysis`, `bounding_box`) only see plain `Vec3`/`Mat3`
  structs and function calls.
- Every stage logs its inputs/outputs via `Logger` (see `logger.hpp`), so a
  run against a stock Eigen and a run against a modified Eigen can be
  diffed line-by-line (`data/perception_sim.log`).
- Every stage is wrapped in a `ScopedTimer` (see `timing.hpp`), so
  performance differences from a modified Eigen are also visible
  (`TimingStats::printSummary()`).
- No attack is simulated anywhere in this code. The project only makes it
  easy to **swap Eigen implementations** and observe the resulting
  difference in outputs — the analysis of what a hypothetical modification
  might do is left entirely to you, outside this codebase.

## Directory layout

```
perception_sim/
    CMakeLists.txt          top-level build config
    include/perception_sim/ public headers (one per pipeline stage)
    src/                    implementations + main.cpp
    data/                   output images / logs / sample point clouds
    scripts/                build.sh, run_pipeline.sh, run_tests.sh
    tests/                  GoogleTest unit tests per module
```

## Building (Ubuntu)

Install dependencies:

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake libeigen3-dev libopencv-dev libpcl-dev
```

Build:

```bash
./scripts/build.sh
```

Run (synthetic point cloud by default):

```bash
./scripts/run_pipeline.sh
# or, with a real .pcd/.ply file:
./scripts/run_pipeline.sh data/my_scan.pcd
```

Output image: `data/output_scene.png`. Full structured log:
`data/perception_sim.log`.

## CSV instrumentation

Every run appends structured, per-event rows to CSV files under
`data/csv/`, each tagged with a `run_id` (timestamp + random suffix,
generated once per process in `CsvLogger::beginRun`). Because rows from
different runs land in the same files, you can diff or filter by `run_id`
to compare a stock-Eigen run against a modified-Eigen run directly.

| File | One row per | Key columns |
|---|---|---|
| `matrix_multiplications.csv` | every matrix-vector multiply in `transformToLocalFrame`/`transformToWorldFrame` (one row per point, plus box center/corners) | `operation`, `m00..m22`, `input_x/y/z`, `output_x/y/z` |
| `centroids.csv` | every call to `computeCentroid` | `num_points`, `x`, `y`, `z` |
| `covariance_matrices.csv` | every call to `computeCovariance` | `num_points`, `c00,c01,c02,c11,c12,c22` (upper triangle; symmetric) |
| `pca_eigenvalues.csv` | every call to `computePca` | `lambda0,lambda1,lambda2` (ascending) |
| `pca_eigenvectors.csv` | every call to `computePca` | `v0_x..v2_z` (columns = eigenvectors, matching `lambda0..lambda2` order) |
| `bounding_boxes.csv` | every finalized `OrientedBoundingBox` | `center_x/y/z`, `axis0..axis2`, `half_extent_x/y/z` |
| `cluster_centers.csv` | every finalized per-object center reported in `PipelineResult` | `num_points`, `x`, `y`, `z` |

All rows share `run_id`, `timestamp_ms`, `row_id` (per-file sequence
number), and a `context` tag (e.g. `cluster_2`, `cluster_2:point_14`,
`cluster_2:corner_5`) so per-point/per-cluster events can be traced back
to a specific object. `centroids.csv`/`covariance_matrices.csv` fire for
*every* centroid/covariance calculation (including the one nested inside
each PCA call), while `cluster_centers.csv` fires only for the finalized,
downstream-facing center of each detected object — the two intentionally
overlap in value but differ in scope, since one is the raw math primitive
and the other is the pipeline's final output.

To compare two runs, run the pipeline twice (optionally with
`EIGEN3_INCLUDE_DIR` pointed at a different Eigen build between runs),
then filter/group each CSV by `run_id` — e.g. with `pandas`:

```python
import pandas as pd
df = pd.read_csv("data/csv/pca_eigenvectors.csv")
baseline = df[df.run_id == "20260703-140501-abc123"]
modified = df[df.run_id == "20260703-141122-def456"]
diff = baseline.merge(modified, on="context", suffixes=("_base", "_mod"))
```

## Comparing two runs (Python)

`scripts/analysis/` contains a self-contained set of Python scripts that
automate exactly that comparison: point them at two `run_id`s and they
produce error metrics, CSV summaries, plots, and a visual overlay.

```bash
# 1. Build + run once against stock Eigen, then again against your
#    modified Eigen (see "Swapping in a modified Eigen" above). Each run
#    gets its own run_id automatically.

# 2. See which run_ids are available:
./scripts/analysis/list_runs.py --csv-dir data/csv

# 3. Compare two of them (or use scripts/compare_runs.sh, which sets up
#    a venv with the required dependencies automatically):
./scripts/compare_runs.sh RUN_ID_A RUN_ID_B
```

This produces `data/comparison/<RUN_ID_A>_vs_<RUN_ID_B>/`, containing:

| Output | What it shows |
|---|---|
| `per_cluster_comparison.csv` | Every metric below, one row per matched cluster pair |
| `unmatched_clusters_run_a.csv` / `_run_b.csv` | Clusters present in only one run (concrete detail behind the cluster-count difference) |
| `aggregate_metrics.csv`, `cluster_count_summary.csv`/`.json` | Mean/max/std per metric, and overall cluster counts |
| `centroid_error_per_cluster.png`, `_histogram.png` | Centroid error: Euclidean distance between matched cluster centers |
| `bbox_center_error_per_cluster.png`, `bbox_size_error_per_cluster.png` | Bounding box error: center distance and per-axis half-extent difference |
| `orientation_error_per_cluster.png`, `_histogram.png` | Orientation error: geodesic angle (degrees) between PCA/box axes, corrected for eigenvector sign ambiguity |
| `eigenvalue_error_per_cluster.png` | PCA eigenvalue drift (bonus signal for root-causing where numerical differences originate) |
| `centroid_vs_orientation_error.png`, `headline_metrics_summary.png` | Cross-metric summary views |
| `overlay_topdown.png` | Top-down visualization overlay: both runs' bounding-box footprints and centroids, with displacement lines between matched clusters and hollow squares marking unmatched (extra/missing) clusters |

**Matching strategy**: clusters are matched between runs by nearest
centroid (Hungarian/optimal assignment on pairwise Euclidean distance), not
by index or context string. This matters because a numerical change could,
via PCL's own internal Eigen dependency (see the dependency analysis),
shift which points survive filtering/clustering upstream of PCA — matching
by index alone would silently misattribute differences if cluster order or
count changed between runs.

Run scripts individually if you want intermediate access (all live in
`scripts/analysis/` and are plain Python, importable from a notebook):
`compute_metrics.py` (metrics only), `generate_report.py` (CSV writers),
`generate_plots.py`, `generate_overlay.py`, `common.py` (shared helpers:
CSV loading, cluster matching, rotation-angle math).


Run tests:

```bash
./scripts/run_tests.sh
```

## Swapping in a modified Eigen

1. Point CMake at your modified Eigen headers:

   ```bash
   EIGEN3_INCLUDE_DIR=/path/to/modified/eigen ./scripts/build.sh
   ```

2. If your modification changes Eigen's API surface (rather than just its
   numerics), the only file you should need to touch is `src/eigen_ops.cpp`.

3. Re-run the pipeline and diff `data/perception_sim.log` /
   `data/output_scene.png` against a baseline run to see how the
   modification propagated through PCA, bounding boxes, and the rendered
   scene.

## Extending

- New filter stage: add to `filtering.hpp/.cpp`, call from `pipeline.cpp`.
- New Eigen-based computation: add a function to `eigen_ops.hpp/.cpp` only;
  keep the rest of the codebase Eigen-free.
- New visualization overlay: extend `visualization.cpp`'s `renderScene`.
