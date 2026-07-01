#pragma once
// -----------------------------------------------------------------------------
// eigen_ops.hpp
//
// *** THE ONLY PLACE IN THIS PROJECT THAT INCLUDES <Eigen/...> HEADERS ***
//
// Design intent
// -------------
// This project exists to study how a modified Eigen propagates through a
// perception pipeline (PCL -> Eigen -> OpenCV). To make that study clean, all
// direct dependence on Eigen types and Eigen algorithms is isolated behind
// this wrapper module.
//
//   * No other header in include/perception_sim/ includes an Eigen header.
//   * No other .cpp file in src/ includes an Eigen header.
//   * Every mathematical operation that PCA / bounding-box / centroid code
//     needs is expressed as a function declared here and implemented in
//     src/eigen_ops.cpp.
//
// To experiment with a modified Eigen, you only need to:
//   1. Point CMake at your modified Eigen include path (see top-level
//      CMakeLists.txt, EIGEN3_INCLUDE_DIR).
//   2. Optionally edit src/eigen_ops.cpp if the modification changes the
//      Eigen API surface used here.
// Nothing else in the codebase needs to know Eigen exists.
// -----------------------------------------------------------------------------

#include <array>
#include <vector>

namespace perception_sim {
namespace eigen_ops {

// A plain (Eigen-free) 3D vector used at the boundary of this module so that
// callers elsewhere in the project never need to spell an Eigen type.
struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

// A plain 3x3 matrix, row-major, used the same way as Vec3 above.
struct Mat3 {
    std::array<double, 9> m{}; // row-major: m[row*3 + col]

    double& at(int row, int col) { return m[static_cast<std::size_t>(row * 3 + col)]; }
    double at(int row, int col) const { return m[static_cast<std::size_t>(row * 3 + col)]; }
};

// Result of a principal component analysis on a point set.
struct PcaResult {
    Vec3 mean;                 // centroid of the input points
    Vec3 eigenvalues;          // ascending order, as returned by Eigen's solver
    Mat3 eigenvectors;         // columns are the corresponding eigenvectors,
                                // eigenvectors.at(row, col) => component `row`
                                // of eigenvector number `col`
};

// Axis-aligned extents of a point set expressed in some local frame,
// used to build oriented bounding boxes once points are rotated into the
// PCA frame.
struct Extent {
    Vec3 min;
    Vec3 max;
};

// ---------------------------------------------------------------------------
// Centroid
// ---------------------------------------------------------------------------
// Computes the arithmetic mean of a point set. Implemented with Eigen
// internally (accumulation via Eigen::Vector3d) so that any numerical
// modification to Eigen's arithmetic is visible here too.
Vec3 computeCentroid(const std::vector<Vec3>& points);

// ---------------------------------------------------------------------------
// Covariance
// ---------------------------------------------------------------------------
// Computes the 3x3 sample covariance matrix of a point set about a given
// mean. This is the direct input to PCA.
Mat3 computeCovariance(const std::vector<Vec3>& points, const Vec3& mean);

// ---------------------------------------------------------------------------
// PCA
// ---------------------------------------------------------------------------
// Runs an eigen-decomposition (via Eigen::SelfAdjointEigenSolver) on the
// covariance of `points`. This is the single most important function to
// watch when studying dependency propagation: any change to Eigen's
// eigen-solver numerics will show up directly in `eigenvalues` /
// `eigenvectors` here, and from there in bounding boxes and visualization.
PcaResult computePca(const std::vector<Vec3>& points);

// ---------------------------------------------------------------------------
// Frame transform
// ---------------------------------------------------------------------------
// Rotates/translates `points` into the local frame defined by `mean` and the
// orthonormal basis `axes` (as produced by computePca). Internally this is a
// matrix-vector multiply per point, done with Eigen.
std::vector<Vec3> transformToLocalFrame(const std::vector<Vec3>& points,
                                         const Vec3& mean,
                                         const Mat3& axes);

// Inverse of transformToLocalFrame: maps a single local-frame point back to
// world coordinates given the same mean/axes.
Vec3 transformToWorldFrame(const Vec3& local_point, const Vec3& mean, const Mat3& axes);

// ---------------------------------------------------------------------------
// Extent / bounding
// ---------------------------------------------------------------------------
// Computes the min/max axis-aligned extent of a point set (typically called
// on points already expressed in a local PCA frame, to build an oriented
// bounding box back in world space).
Extent computeExtent(const std::vector<Vec3>& points);

// ---------------------------------------------------------------------------
// Misc small helpers
// ---------------------------------------------------------------------------
// Euclidean distance between two points, used by clustering code without
// that code needing to include Eigen directly.
double distance(const Vec3& a, const Vec3& b);

} // namespace eigen_ops
} // namespace perception_sim
