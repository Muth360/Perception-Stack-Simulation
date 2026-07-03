#pragma once
// -----------------------------------------------------------------------------
// csv_logger.hpp
//
// Structured CSV instrumentation for every numerically interesting operation
// in the pipeline. Distinct from Logger (human-readable text log): this
// writes one row per event to a dedicated CSV file per category, tagged with
// a run_id, so multiple runs (e.g. stock Eigen vs. a modified Eigen) can be
// appended to the same files and compared/diffed by run_id afterward.
//
// Depends only on eigen_ops.hpp for the Vec3/Mat3 plain data types - it does
// NOT include any Eigen header itself, preserving the isolation boundary
// described in eigen_ops.hpp.
//
// Categories (one CSV file each, under the configured output directory):
//   matrix_multiplications.csv  - every matrix-vector multiply (frame transforms)
//   centroids.csv                - every centroid calculation
//   covariance_matrices.csv      - every covariance matrix
//   pca_eigenvalues.csv          - every PCA eigenvalue triple
//   pca_eigenvectors.csv         - every PCA eigenvector basis (3x3)
//   bounding_boxes.csv           - every oriented bounding box
//   cluster_centers.csv          - every finalized per-object cluster center
// -----------------------------------------------------------------------------

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

    // Sets the output directory (created if missing) and generates a fresh
    // run_id for subsequent log calls. Call once near the start of main().
    void beginRun(const std::string& output_dir = "data/csv");

    const std::string& runId() const { return run_id_; }

    // --- Logging methods, one per instrumented category -----------------

    // Matrix-vector multiplication, e.g. R^T*(p-mean) or R*local+mean.
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

    // Returns (creating/opening if necessary) the file for `category`,
    // writing `header` as the first line if the file is newly created.
    CsvFile& fileFor(const std::string& category, const std::string& header);

    // Wall-clock milliseconds since epoch, for the timestamp_ms column.
    static long long nowMs();

    std::mutex mutex_;
    std::string output_dir_ = "data/csv";
    std::string run_id_ = "unset_run";
    std::map<std::string, std::unique_ptr<CsvFile>> files_;
};

} // namespace perception_sim
