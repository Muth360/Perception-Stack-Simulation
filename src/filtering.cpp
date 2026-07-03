#include "perception_sim/filtering.hpp"
#include "perception_sim/logger.hpp"
#include "perception_sim/timing.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

namespace perception_sim {
namespace filtering {

CloudPtr voxelFilter(const CloudConstPtr& input, const VoxelFilterParams& params) {
    ScopedTimer timer("filtering.voxel");
    CloudPtr output(new CloudT);

    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(input);
    voxel.setLeafSize(params.leaf_size_x, params.leaf_size_y, params.leaf_size_z);
    voxel.filter(*output);

    Logger::instance().info("filtering",
        LogFields().add("stage", "voxel")
                    .add("input_points", input->size())
                    .add("output_points", output->size())
                    .add("leaf_x", params.leaf_size_x)
                    .add("leaf_y", params.leaf_size_y)
                    .add("leaf_z", params.leaf_size_z).str());
    return output;
}

CloudPtr passThroughFilter(const CloudConstPtr& input, const PassThroughParams& params) {
    ScopedTimer timer("filtering.passthrough");
    CloudPtr output(new CloudT);

    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(input);
    pass.setFilterFieldName(params.field_name);
    pass.setFilterLimits(params.min_limit, params.max_limit);
    pass.filter(*output);

    Logger::instance().info("filtering",
        LogFields().add("stage", "passthrough")
                    .add("field", params.field_name)
                    .add("input_points", input->size())
                    .add("output_points", output->size())
                    .add("min", params.min_limit)
                    .add("max", params.max_limit).str());
    return output;
}

} // namespace filtering
} // namespace perception_sim
