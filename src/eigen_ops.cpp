// -----------------------------------------------------------------------------
// eigen_ops.cpp
//
// The only translation unit in this project that includes <Eigen/...>.
// See eigen_ops.hpp for the design rationale.
// -----------------------------------------------------------------------------
#include "perception_sim/eigen_ops.hpp"
#include "perception_sim/csv_logger.hpp"

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

Vec3 computeCentroid(const std::vector<Vec3>& points, const std::string& context) {
    if (points.empty()) {
        const Vec3 zero{};
        CsvLogger::instance().logCentroid(0, zero, context);
        return zero;
    }

    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    for (const auto& p : points) {
        sum += toEigen(p);
    }
    sum /= static_cast<double>(points.size());
    const Vec3 result = fromEigen(sum);

    CsvLogger::instance().logCentroid(points.size(), result, context);
    return result;
}

Mat3 computeCovariance(const std::vector<Vec3>& points, const Vec3& mean, const std::string& context) {
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    if (points.size() < 2) {
        const Mat3 result = fromEigen(cov);
        CsvLogger::instance().logCovariance(points.size(), result, context);
        return result;
    }

    const Eigen::Vector3d mu = toEigen(mean);
    for (const auto& p : points) {
        const Eigen::Vector3d d = toEigen(p) - mu;
        cov += d * d.transpose();
    }
    cov /= static_cast<double>(points.size() - 1);
    const Mat3 result = fromEigen(cov);

    CsvLogger::instance().logCovariance(points.size(), result, context);
    return result;
}

PcaResult computePca(const std::vector<Vec3>& points, const std::string& context) {
    PcaResult result;
    result.mean = computeCentroid(points, context);

    const Mat3 cov = computeCovariance(points, result.mean, context);
    const Eigen::Matrix3d covE = toEigen(cov);

    // SelfAdjointEigenSolver is appropriate because covariance matrices are
    // symmetric positive-semidefinite. This is the core numerical routine
    // whose behavior changes if the linked Eigen implementation changes.
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covE);

    const Eigen::Vector3d eigenvalues = solver.eigenvalues();   // ascending
    const Eigen::Matrix3d eigenvectors = solver.eigenvectors(); // columns

    result.eigenvalues = fromEigen(eigenvalues);
    result.eigenvectors = fromEigen(eigenvectors);

    CsvLogger::instance().logPcaEigenvalues(result.eigenvalues, context);
    CsvLogger::instance().logPcaEigenvectors(result.eigenvectors, context);
    return result;
}

std::vector<Vec3> transformToLocalFrame(const std::vector<Vec3>& points,
                                         const Vec3& mean,
                                         const Mat3& axes,
                                         const std::string& context) {
    const Eigen::Vector3d mu = toEigen(mean);
    const Eigen::Matrix3d R = toEigen(axes); // columns = axes
    const Eigen::Matrix3d Rt = R.transpose();
    const Mat3 R_transpose = fromEigen(Rt);

    std::vector<Vec3> out;
    out.reserve(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
        // Local coordinates = R^T * (p - mean); R is orthonormal so R^T = R^-1.
        const Eigen::Vector3d centered = toEigen(points[i]) - mu;
        const Eigen::Vector3d local = R.transpose() * centered;
        const Vec3 local_v3 = fromEigen(local);
        out.push_back(local_v3);

        CsvLogger::instance().logMatrixMultiplication(
            "local_frame_transform", R_transpose, fromEigen(centered), local_v3,
            context.empty() ? ("point_" + std::to_string(i)) : (context + ":point_" + std::to_string(i)));
    }
    return out;
}

Vec3 transformToWorldFrame(const Vec3& local_point, const Vec3& mean, const Mat3& axes, const std::string& context) {
    const Eigen::Vector3d mu = toEigen(mean);
    const Eigen::Matrix3d R = toEigen(axes);
    const Eigen::Vector3d rotated = R * toEigen(local_point);
    const Eigen::Vector3d world = rotated + mu;
    const Vec3 world_v3 = fromEigen(world);

    CsvLogger::instance().logMatrixMultiplication(
        "world_frame_transform", axes, local_point, fromEigen(rotated), context);

    return world_v3;
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
