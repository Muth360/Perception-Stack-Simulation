#pragma once
// -----------------------------------------------------------------------------
// pca_analysis.hpp
//
// Runs PCA on a cluster's points to obtain a centroid + principal axes.
// This module is deliberately Eigen-free at the API/implementation level: it
// converts PCL points into perception_sim::eigen_ops::Vec3 and delegates all
// linear algebra to eigen_ops. This is the "isolation boundary" in action.
// -----------------------------------------------------------------------------

#include "perception_sim/types.hpp"
#include "perception_sim/eigen_ops.hpp"

namespace perception_sim {
namespace pca {

struct ClusterPcaResult {
    eigen_ops::Vec3 centroid;
    eigen_ops::Vec3 eigenvalues;
    eigen_ops::Mat3 eigenvectors;
};

// Computes centroid + PCA eigenvectors/eigenvalues for a single cluster.
// `cluster_index` is used purely to tag CSV instrumentation rows (see
// csv_logger.hpp) with a human-readable context like "cluster_2".
ClusterPcaResult analyzeCluster(const Cluster& cluster, std::size_t cluster_index = 0);

} // namespace pca
} // namespace perception_sim
