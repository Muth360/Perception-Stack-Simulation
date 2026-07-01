#include <gtest/gtest.h>
#include "perception_sim/filtering.hpp"

using namespace perception_sim;

namespace {
CloudPtr makeGridCloud() {
    CloudPtr cloud(new CloudT);
    for (float x = 0.0f; x < 1.0f; x += 0.02f) {
        for (float y = 0.0f; y < 1.0f; y += 0.02f) {
            PointT p; p.x = x; p.y = y; p.z = 0.0f;
            cloud->push_back(p);
        }
    }
    cloud->width = static_cast<uint32_t>(cloud->size());
    cloud->height = 1;
    return cloud;
}
} // namespace

TEST(Filtering, VoxelFilterReducesPointCount) {
    CloudPtr cloud = makeGridCloud();
    filtering::VoxelFilterParams params;
    params.leaf_size_x = params.leaf_size_y = params.leaf_size_z = 0.2f;

    CloudPtr out = filtering::voxelFilter(cloud, params);
    EXPECT_LT(out->size(), cloud->size());
    EXPECT_GT(out->size(), 0u);
}

TEST(Filtering, PassThroughRemovesOutOfRangePoints) {
    CloudPtr cloud(new CloudT);
    for (int i = -10; i <= 10; ++i) {
        PointT p; p.x = 0; p.y = 0; p.z = static_cast<float>(i);
        cloud->push_back(p);
    }
    cloud->width = static_cast<uint32_t>(cloud->size());
    cloud->height = 1;

    filtering::PassThroughParams params;
    params.field_name = "z";
    params.min_limit = -2.0f;
    params.max_limit = 2.0f;

    CloudPtr out = filtering::passThroughFilter(cloud, params);
    EXPECT_EQ(out->size(), 5u); // z in {-2,-1,0,1,2}
    for (const auto& p : out->points) {
        EXPECT_GE(p.z, -2.0f);
        EXPECT_LE(p.z, 2.0f);
    }
}
