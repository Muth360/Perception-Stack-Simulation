#pragma once
// -----------------------------------------------------------------------------
// filtering.hpp
//
// PCL-based preprocessing filters: voxel-grid downsampling and passthrough
// (range) filtering. No Eigen usage here - PCL's filter classes handle their
// own internals.
// -----------------------------------------------------------------------------

#include "perception_sim/types.hpp"

namespace perception_sim {
namespace filtering {

struct VoxelFilterParams {
    float leaf_size_x = 0.1f;
    float leaf_size_y = 0.1f;
    float leaf_size_z = 0.1f;
};

struct PassThroughParams {
    std::string field_name = "z";
    float min_limit = -5.0f;
    float max_limit = 5.0f;
};

// Downsamples `input` using a voxel grid filter. Returns a new cloud.
CloudPtr voxelFilter(const CloudConstPtr& input, const VoxelFilterParams& params);

// Removes points outside [min_limit, max_limit] along `field_name`.
CloudPtr passThroughFilter(const CloudConstPtr& input, const PassThroughParams& params);

} // namespace filtering
} // namespace perception_sim
