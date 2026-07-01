#pragma once
// -----------------------------------------------------------------------------
// clustering.hpp
//
// Euclidean cluster extraction via PCL's KdTree + EuclideanClusterExtraction.
// -----------------------------------------------------------------------------

#include "perception_sim/types.hpp"

namespace perception_sim {
namespace clustering {

struct ClusterParams {
    double cluster_tolerance = 0.5; // meters, max distance between neighbor points
    int min_cluster_size = 5;
    int max_cluster_size = 25000;
};

// Splits `input` into clusters. Each returned Cluster owns its own point
// subset cloud.
ClusterVec extractClusters(const CloudConstPtr& input, const ClusterParams& params);

} // namespace clustering
} // namespace perception_sim
