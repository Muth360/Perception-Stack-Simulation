#include <gtest/gtest.h>
#include "perception_sim/eigen_ops.hpp"

using namespace perception_sim::eigen_ops;

TEST(EigenOps, CentroidOfSinglePoint) {
    std::vector<Vec3> pts = {Vec3{1.0, 2.0, 3.0}};
    const Vec3 c = computeCentroid(pts);
    EXPECT_DOUBLE_EQ(c.x, 1.0);
    EXPECT_DOUBLE_EQ(c.y, 2.0);
    EXPECT_DOUBLE_EQ(c.z, 3.0);
}

TEST(EigenOps, CentroidOfSymmetricPoints) {
    std::vector<Vec3> pts = {
        Vec3{-1.0, 0.0, 0.0},
        Vec3{1.0, 0.0, 0.0},
        Vec3{0.0, -1.0, 0.0},
        Vec3{0.0, 1.0, 0.0},
    };
    const Vec3 c = computeCentroid(pts);
    EXPECT_NEAR(c.x, 0.0, 1e-9);
    EXPECT_NEAR(c.y, 0.0, 1e-9);
    EXPECT_NEAR(c.z, 0.0, 1e-9);
}

TEST(EigenOps, EmptyCentroidIsZero) {
    std::vector<Vec3> pts;
    const Vec3 c = computeCentroid(pts);
    EXPECT_DOUBLE_EQ(c.x, 0.0);
    EXPECT_DOUBLE_EQ(c.y, 0.0);
    EXPECT_DOUBLE_EQ(c.z, 0.0);
}

TEST(EigenOps, PcaOnLineSegmentAlignsWithPrimaryAxis) {
    // Points along the x-axis: dominant eigenvector should be ~(±1,0,0),
    // with the largest eigenvalue in the last column (ascending order).
    std::vector<Vec3> pts;
    for (double x = -5.0; x <= 5.0; x += 1.0) {
        pts.push_back(Vec3{x, 0.0, 0.0});
    }

    const PcaResult result = computePca(pts);

    // Largest eigenvalue is at index 2 (SelfAdjointEigenSolver returns ascending order).
    EXPECT_GT(result.eigenvalues.z, result.eigenvalues.x);
    EXPECT_GT(result.eigenvalues.z, result.eigenvalues.y);

    const double vx = std::abs(result.eigenvectors.at(0, 2));
    const double vy = std::abs(result.eigenvectors.at(1, 2));
    const double vz = std::abs(result.eigenvectors.at(2, 2));
    EXPECT_NEAR(vx, 1.0, 1e-6);
    EXPECT_NEAR(vy, 0.0, 1e-6);
    EXPECT_NEAR(vz, 0.0, 1e-6);
}

TEST(EigenOps, TransformRoundTrip) {
    Mat3 identity;
    identity.at(0, 0) = 1.0; identity.at(1, 1) = 1.0; identity.at(2, 2) = 1.0;

    const Vec3 mean{1.0, 2.0, 3.0};
    std::vector<Vec3> pts = {Vec3{4.0, 5.0, 6.0}};

    const std::vector<Vec3> local = transformToLocalFrame(pts, mean, identity);
    ASSERT_EQ(local.size(), 1u);
    EXPECT_NEAR(local[0].x, 3.0, 1e-9);
    EXPECT_NEAR(local[0].y, 3.0, 1e-9);
    EXPECT_NEAR(local[0].z, 3.0, 1e-9);

    const Vec3 back = transformToWorldFrame(local[0], mean, identity);
    EXPECT_NEAR(back.x, pts[0].x, 1e-9);
    EXPECT_NEAR(back.y, pts[0].y, 1e-9);
    EXPECT_NEAR(back.z, pts[0].z, 1e-9);
}

TEST(EigenOps, ExtentComputesMinMax) {
    std::vector<Vec3> pts = {
        Vec3{-2.0, 3.0, 0.0},
        Vec3{5.0, -1.0, 10.0},
        Vec3{0.0, 0.0, 0.0},
    };
    const Extent e = computeExtent(pts);
    EXPECT_DOUBLE_EQ(e.min.x, -2.0);
    EXPECT_DOUBLE_EQ(e.min.y, -1.0);
    EXPECT_DOUBLE_EQ(e.min.z, 0.0);
    EXPECT_DOUBLE_EQ(e.max.x, 5.0);
    EXPECT_DOUBLE_EQ(e.max.y, 3.0);
    EXPECT_DOUBLE_EQ(e.max.z, 10.0);
}

TEST(EigenOps, DistanceIsEuclidean) {
    const Vec3 a{0.0, 0.0, 0.0};
    const Vec3 b{3.0, 4.0, 0.0};
    EXPECT_NEAR(distance(a, b), 5.0, 1e-9);
}
