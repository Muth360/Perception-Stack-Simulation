#pragma once
// -----------------------------------------------------------------------------
// types.hpp
//
// Common type aliases used throughout perception_sim. Keeping these in one
// place makes it easy to see everywhere the project touches PCL point types
// or Eigen types, which is useful when reasoning about dependency propagation.
// -----------------------------------------------------------------------------

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

namespace perception_sim {

using PointT      = pcl::PointXYZ;
using CloudT       = pcl::PointCloud<PointT>;
using CloudPtr      = CloudT::Ptr;
using CloudConstPtr  = CloudT::ConstPtr;

/// A single detected cluster: the raw points that belong to it.
struct Cluster {
    CloudPtr points;
};

using ClusterVec = std::vector<Cluster>;

} // namespace perception_sim
