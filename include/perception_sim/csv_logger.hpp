#pragma once

#include "perception_sim/eigen_ops.hpp"

#include <atomic>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <string>

namespace perception_sim {

class CsvLogger {
public:
    static CsvLogger& instance();

    // Creates a new run directory and run_id
    void beginRun(const std::string& output_dir = "data/runs");

    const std::string& runId() const { return run_id_; }
    const std::string& runDir() const { return run_dir_; }

    // ---------------------------------------------------------------------
    // Logging APIs
    // ---------------------------------------------------------------------

    void logMatrixMultiplication(const std::string& operation,
                                  const eigen_ops::Mat3& matrix,
                                  const eigen_ops::Vec3& input,
                                  const eigen_ops::Vec3& output,
                                  const std::string& context);

    void logCentroid(std::size_t num_points,
                      const eigen_ops::Vec3& centroid,
                      const std::string& context);

    void logCovariance(std::size_t num_points,
                        const eigen_ops::Mat3& covariance,
                        const std::string& context);

    void logPcaEigenvalues(const eigen_ops::Vec3& eigenvalues,
                            const std::string& context);

    void logPcaEigenvectors(const eigen_ops::Mat3& eigenvectors,
                             const std::string& context);

    void logBoundingBox(const eigen_ops::Vec3& center,
                         const eigen_ops::Mat3& axes,
                         const eigen_ops::Vec3& half_extents,
                         const std::string& context);

    void logClusterCenter(std::size_t num_points,
                           const eigen_ops::Vec3& center,
                           const std::string& context);

private:
    CsvLogger() = default;

    struct CsvFile {
        std::ofstream stream;
        bool header_written = false;
        std::atomic<std::uint64_t> next_row_id{0};
    };

    CsvFile& fileFor(const std::string& category, const std::string& header);

    static long long nowMs();

    std::mutex mutex_;

    // ---------------------------------------------------------------------
    // FIX: run-based isolation (this is what was missing)
    // ---------------------------------------------------------------------
    std::string output_dir_ = "data/runs";
    std::string run_dir_;
    std::string run_id_ = "unset_run";

    std::map<std::string, std::unique_ptr<CsvFile>> files_;
};

} // namespace perception_sim
