// -----------------------------------------------------------------------------
// eigen_ops.cpp
//
// The only translation unit in this project that includes <Eigen/...>.
// See eigen_ops.hpp for the design rationale.
// -----------------------------------------------------------------------------
#include "perception_sim/eigen_ops.hpp"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

namespace perception_sim {
namespace eigen_ops {

namespace {

inline Eigen::Vector3d toEigen(const Vec3& v) {
    return Eigen::Vector3d(v.x, v.y, v.z);
}

inline Vec3 fromEigen(const Eigen::Vector3d& v) {
    return Vec3{v.x(), v.y(), v.z()};
}

inline Eigen::Matrix3d toEigen(const Mat3& m) {
    Eigen::Matrix3d out;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            out(r, c) = m.at(r, c);
    return out;
}

inline Mat3 fromEigen(const Eigen::Matrix3d& m) {
    Mat3 out;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            out.at(r, c) = m(r, c);
    return out;
}

} // namespace

Vec3 computeCentroid(const std::vector<Vec3>& points) {
    if (points.empty()) return Vec3{};

    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    for (const auto& p : points) {
        sum += toEigen(p);
    }
    sum /= static_cast<double>(points.size());
    return fromEigen(sum);
}

Mat3 computeCovariance(const std::vector<Vec3>& points, const Vec3& mean) {
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    if (points.size() < 2) return fromEigen(cov);

    const Eigen::Vector3d mu = toEigen(mean);
    for (const auto& p : points) {
        const Eigen::Vector3d d = toEigen(p) - mu;
        cov += d * d.transpose();
    }
    cov /= static_cast<double>(points.size() - 1);
    return fromEigen(cov);
}

PcaResult computePca(const std::vector<Vec3>& points) {
    PcaResult result;
    result.mean = computeCentroid(points);

    const Mat3 cov = computeCovariance(points, result.mean);
    const Eigen::Matrix3d covE = toEigen(cov);

    // SelfAdjointEigenSolver is appropriate because covariance matrices are
    // symmetric positive-semidefinite. This is the core numerical routine
    // whose behavior changes if the linked Eigen implementation changes.
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covE);

    const Eigen::Vector3d eigenvalues = solver.eigenvalues();   // ascending
    const Eigen::Matrix3d eigenvectors = solver.eigenvectors(); // columns

    result.eigenvalues = fromEigen(eigenvalues);
    result.eigenvectors = fromEigen(eigenvectors);
    return result;
}

std::vector<Vec3> transformToLocalFrame(const std::vector<Vec3>& points,
                                         const Vec3& mean,
                                         const Mat3& axes) {
    const Eigen::Vector3d mu = toEigen(mean);
    const Eigen::Matrix3d R = toEigen(axes); // columns = axes

    std::vector<Vec3> out;
    out.reserve(points.size());
    for (const auto& p : points) {
        // Local coordinates = R^T * (p - mean); R is orthonormal so R^T = R^-1.
        const Eigen::Vector3d local = R.transpose() * (toEigen(p) - mu);
        out.push_back(fromEigen(local));
    }
    return out;
}

Vec3 transformToWorldFrame(const Vec3& local_point, const Vec3& mean, const Mat3& axes) {
    const Eigen::Vector3d mu = toEigen(mean);
    const Eigen::Matrix3d R = toEigen(axes);
    const Eigen::Vector3d world = R * toEigen(local_point) + mu;
    return fromEigen(world);
}

Extent computeExtent(const std::vector<Vec3>& points) {
    Extent e;
    if (points.empty()) return e;

    Eigen::Vector3d mn = toEigen(points.front());
    Eigen::Vector3d mx = mn;
    for (const auto& p : points) {
        const Eigen::Vector3d v = toEigen(p);
        mn = mn.cwiseMin(v);
        mx = mx.cwiseMax(v);
    }
    e.min = fromEigen(mn);
    e.max = fromEigen(mx);
    return e;
}

double distance(const Vec3& a, const Vec3& b) {
    return (toEigen(a) - toEigen(b)).norm();
}

} // namespace eigen_ops
} // namespace perception_sim
