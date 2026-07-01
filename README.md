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

