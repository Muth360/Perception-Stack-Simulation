#include <gtest/gtest.h>
#include "perception_sim/clustering.hpp"

using namespace perception_sim;

TEST(Clustering, SeparatesTwoDistinctGroups) {
    CloudPtr cloud(new CloudT);

    // Group A near origin.
    for (int i = 0; i < 10; ++i) {
        PointT p; p.x = 0.0f + 0.01f * i; p.y = 0.0f; p.z = 0.0f;
        cloud->push_back(p);
    }
    // Group B far away.
    for (int i = 0; i < 10; ++i) {
        PointT p; p.x = 100.0f + 0.01f * i; p.y = 0.0f; p.z = 0.0f;
        cloud->push_back(p);
    }
    cloud->width = static_cast<uint32_t>(cloud->size());
    cloud->height = 1;

    clustering::ClusterParams params;
    params.cluster_tolerance = 1.0;
    params.min_cluster_size = 1;

    ClusterVec clusters = clustering::extractClusters(cloud, params);
    EXPECT_EQ(clusters.size(), 2u);
}

TEST(Clustering, EmptyInputProducesNoClusters) {
    CloudPtr cloud(new CloudT);
    clustering::ClusterParams params;
    ClusterVec clusters = clustering::extractClusters(cloud, params);
    EXPECT_TRUE(clusters.empty());
}

TEST(Clustering, MinClusterSizeFiltersSmallGroups) {
    CloudPtr cloud(new CloudT);
    // A group of only 2 points, below a min_cluster_size of 5.
    for (int i = 0; i < 2; ++i) {
        PointT p; p.x = static_cast<float>(i) * 0.01f; p.y = 0.0f; p.z = 0.0f;
        cloud->push_back(p);
    }
    cloud->width = static_cast<uint32_t>(cloud->size());
    cloud->height = 1;

    clustering::ClusterParams params;
    params.cluster_tolerance = 1.0;
    params.min_cluster_size = 5;

    ClusterVec clusters = clustering::extractClusters(cloud, params);
    EXPECT_TRUE(clusters.empty());
}
