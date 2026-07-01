#pragma once
// -----------------------------------------------------------------------------
// bounding_box.hpp
//
// Builds an oriented bounding box (OBB) for a cluster from its PCA result.
// Like pca_analysis, this stays Eigen-free and routes all linear algebra
// through eigen_ops.
// -----------------------------------------------------------------------------

#include "perception_sim/types.hpp"
#include "perception_sim/pca_analysis.hpp"
#include "perception_sim/eigen_ops.hpp"
#include <array>

namespace perception_sim {
namespace bbox {

struct OrientedBoundingBox {
    eigen_ops::Vec3 center;               // box center in world coordinates
    eigen_ops::Mat3 axes;                 // orientation (columns = box axes)
    eigen_ops::Vec3 half_extents;         // half-size along each axis
    std::array<eigen_ops::Vec3, 8> corners; // world-space corners
};

// Computes an oriented bounding box for `cluster` using a precomputed PCA
// result (centroid + principal axes) for that same cluster.
OrientedBoundingBox computeOrientedBoundingBox(const Cluster& cluster,
                                                const pca::ClusterPcaResult& pca_result);

} // namespace bbox
} // namespace perception_sim
