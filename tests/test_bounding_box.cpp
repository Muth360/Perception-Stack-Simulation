#include <gtest/gtest.h>
#include "perception_sim/pca_analysis.hpp"
#include "perception_sim/bounding_box.hpp"

using namespace perception_sim;

TEST(BoundingBox, AxisAlignedCubeGivesExpectedHalfExtents) {
    CloudPtr cloud(new CloudT);
    for (float x : {-1.0f, 1.0f}) {
        for (float y : {-1.0f, 1.0f}) {
            for (float z : {-1.0f, 1.0f}) {
                PointT p; p.x = x; p.y = y; p.z = z;
                cloud->push_back(p);
            }
        }
    }
    cloud->width = static_cast<uint32_t>(cloud->size());
    cloud->height = 1;

    Cluster cluster{cloud};
    pca::ClusterPcaResult pca_result = pca::analyzeCluster(cluster);
    bbox::OrientedBoundingBox box = bbox::computeOrientedBoundingBox(cluster, pca_result);

    // For a symmetric cube centered at origin, half extents along each PCA
    // axis should all be ~1.0.
    EXPECT_NEAR(box.half_extents.x, 1.0, 1e-5);
    EXPECT_NEAR(box.half_extents.y, 1.0, 1e-5);
    EXPECT_NEAR(box.half_extents.z, 1.0, 1e-5);

    EXPECT_NEAR(box.center.x, 0.0, 1e-5);
    EXPECT_NEAR(box.center.y, 0.0, 1e-5);
    EXPECT_NEAR(box.center.z, 0.0, 1e-5);
}

TEST(BoundingBox, HasEightUniqueCorners) {
    CloudPtr cloud(new CloudT);
    for (float x : {0.0f, 2.0f}) {
        for (float y : {0.0f, 3.0f}) {
            for (float z : {0.0f, 1.0f}) {
                PointT p; p.x = x; p.y = y; p.z = z;
                cloud->push_back(p);
            }
        }
    }
    cloud->width = static_cast<uint32_t>(cloud->size());
    cloud->height = 1;

    Cluster cluster{cloud};
    pca::ClusterPcaResult pca_result = pca::analyzeCluster(cluster);
    bbox::OrientedBoundingBox box = bbox::computeOrientedBoundingBox(cluster, pca_result);

    for (std::size_t i = 0; i < box.corners.size(); ++i) {
        for (std::size_t j = i + 1; j < box.corners.size(); ++j) {
            const double dx = box.corners[i].x - box.corners[j].x;
            const double dy = box.corners[i].y - box.corners[j].y;
            const double dz = box.corners[i].z - box.corners[j].z;
            const double dist2 = dx * dx + dy * dy + dz * dz;
            EXPECT_GT(dist2, 1e-9);
        }
    }
}
