#pragma once
// -----------------------------------------------------------------------------
// point_cloud_io.hpp
//
// Loading (or synthetic generation) of the input point cloud. Uses PCL's I/O
// facilities only; no Eigen usage here.
// -----------------------------------------------------------------------------

#include "perception_sim/types.hpp"
#include <string>

namespace perception_sim {
namespace io {

// Loads a point cloud from a .pcd or .ply file on disk.
// Throws std::runtime_error on failure.
CloudPtr loadPointCloud(const std::string& path);

// Generates a synthetic point cloud with a handful of clustered "objects"
// plus optional background noise, useful when no real dataset is on hand.
// `num_objects` clusters of Gaussian-distributed points are placed at random
// positions within [-spread, spread] on x/y, z near ground level.
CloudPtr generateSyntheticCloud(int num_objects = 4,
                                 int points_per_object = 200,
                                 double spread = 10.0,
                                 unsigned int random_seed = 42);

// Writes a point cloud to disk in PCD format (ASCII), primarily for
// debugging/inspection between pipeline runs.
void savePointCloud(const std::string& path, const CloudT& cloud);

} // namespace io
} // namespace perception_sim
