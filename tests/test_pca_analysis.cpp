#include <gtest/gtest.h>
#include "perception_sim/pca_analysis.hpp"

using namespace perception_sim;

TEST(PcaAnalysis, CentroidMatchesSimpleMean) {
    CloudPtr cloud(new CloudT);
    PointT p1; p1.x = 0; p1.y = 0; p1.z = 0;
    PointT p2; p2.x = 2; p2.y = 0; p2.z = 0;
    PointT p3; p3.x = 1; p3.y = 2; p3.z = 0;
    cloud->push_back(p1);
    cloud->push_back(p2);
    cloud->push_back(p3);
    cloud->width = 3;
    cloud->height = 1;

    Cluster cluster{cloud};
    pca::ClusterPcaResult result = pca::analyzeCluster(cluster);

    EXPECT_NEAR(result.centroid.x, 1.0, 1e-6);
    EXPECT_NEAR(result.centroid.y, 2.0 / 3.0, 1e-6);
    EXPECT_NEAR(result.centroid.z, 0.0, 1e-6);
}

TEST(PcaAnalysis, FlatClusterHasNearZeroSmallestEigenvalue) {
    // All points on the z=0 plane -> smallest eigenvalue should be ~0.
    CloudPtr cloud(new CloudT);
    for (float x = -2.0f; x <= 2.0f; x += 1.0f) {
        for (float y = -2.0f; y <= 2.0f; y += 1.0f) {
            PointT p; p.x = x; p.y = y; p.z = 0.0f;
            cloud->push_back(p);
        }
    }
    cloud->width = static_cast<uint32_t>(cloud->size());
    cloud->height = 1;

    Cluster cluster{cloud};
    pca::ClusterPcaResult result = pca::analyzeCluster(cluster);

    // Ascending order -> smallest is .x
    EXPECT_NEAR(result.eigenvalues.x, 0.0, 1e-6);
}
