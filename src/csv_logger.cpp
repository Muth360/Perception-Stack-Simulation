#include "perception_sim/csv_logger.hpp"
#include "perception_sim/logger.hpp"

#include <chrono>
#include <filesystem>
#include <iomanip>
#include <random>
#include <sstream>

namespace perception_sim {

namespace {

std::string generateRunId() {
    const auto now = std::chrono::system_clock::now();
    const auto t = std::chrono::system_clock::to_time_t(now);

    std::ostringstream oss;
    oss << std::put_time(std::localtime(&t), "%Y%m%d-%H%M%S");

    std::random_device rd;
    std::mt19937 rng(rd());
    std::uniform_int_distribution<int> dist(0, 15);
    static const char* hex = "0123456789abcdef";

    oss << "-";
    for (int i = 0; i < 6; ++i)
        oss << hex[dist(rng)];

    return oss.str();
}

} // namespace


// ============================================================================
// Singleton
// ============================================================================
CsvLogger& CsvLogger::instance() {
    static CsvLogger logger;
    return logger;
}


// ============================================================================
// Time helper
// ============================================================================
long long CsvLogger::nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();
}


// ============================================================================
// Begin run (CRITICAL FIX HERE)
// ============================================================================
void CsvLogger::beginRun(const std::string& output_dir) {
    std::lock_guard<std::mutex> lock(mutex_);

    run_id_ = generateRunId();

    run_dir_ = output_dir + "/" + run_id_;
    std::filesystem::create_directories(run_dir_);

    files_.clear();

    Logger::instance().info(
        "csv_logger",
        LogFields()
            .add("action", "begin_run")
            .add("run_id", run_id_)
            .add("run_dir", run_dir_)
            .str()
    );
}


// ============================================================================
// File manager (FIXED: per-run filenames)
// ============================================================================
CsvLogger::CsvFile& CsvLogger::fileFor(const std::string& category,
                                       const std::string& header) {
    auto it = files_.find(category);
    if (it != files_.end())
        return *it->second;

    std::filesystem::create_directories(run_dir_);

    auto file = std::make_unique<CsvFile>();

    // ============================================================
    // KEY FIX: filename includes run_id AND run folder
    // ============================================================
    const std::string path =
        run_dir_ + "/" + category + "_" + run_id_ + ".csv";

    const bool exists =
        std::filesystem::exists(path) &&
        std::filesystem::file_size(path) > 0;

    file->stream.open(path, std::ios::out | std::ios::app);

    file->header_written = exists;

    if (!file->header_written) {
        file->stream << header << "\n";
        file->header_written = true;
    }

    auto [it2, ok] = files_.emplace(category, std::move(file));
    (void)ok;

    return *it2->second;
}


// ============================================================================
// Matrix multiplication log
// ============================================================================
void CsvLogger::logMatrixMultiplication(const std::string& operation,
                                         const eigen_ops::Mat3& matrix,
                                         const eigen_ops::Vec3& input,
                                         const eigen_ops::Vec3& output,
                                         const std::string& context) {
    std::lock_guard<std::mutex> lock(mutex_);

    CsvFile& f = fileFor(
        "matrix_multiplications",
        "run_id,timestamp_ms,row_id,context,operation,"
        "m00,m01,m02,m10,m11,m12,m20,m21,m22,"
        "input_x,input_y,input_z,output_x,output_y,output_z"
    );

    const auto row_id = f.next_row_id.fetch_add(1);

    f.stream
        << run_id_ << ","
        << nowMs() << ","
        << row_id << ","
        << context << ","
        << operation << ","
        << matrix.at(0,0) << "," << matrix.at(0,1) << "," << matrix.at(0,2) << ","
        << matrix.at(1,0) << "," << matrix.at(1,1) << "," << matrix.at(1,2) << ","
        << matrix.at(2,0) << "," << matrix.at(2,1) << "," << matrix.at(2,2) << ","
        << input.x << "," << input.y << "," << input.z << ","
        << output.x << "," << output.y << "," << output.z
        << "\n";

    f.stream.flush();
}


// ============================================================================
// Centroid
// ============================================================================
void CsvLogger::logCentroid(std::size_t num_points,
                             const eigen_ops::Vec3& centroid,
                             const std::string& context) {
    std::lock_guard<std::mutex> lock(mutex_);

    CsvFile& f = fileFor(
        "centroids",
        "run_id,timestamp_ms,row_id,context,num_points,x,y,z"
    );

    const auto row_id = f.next_row_id.fetch_add(1);

    f.stream
        << run_id_ << ","
        << nowMs() << ","
        << row_id << ","
        << context << ","
        << num_points << ","
        << centroid.x << ","
        << centroid.y << ","
        << centroid.z
        << "\n";

    f.stream.flush();
}


// ============================================================================
// Covariance
// ============================================================================
void CsvLogger::logCovariance(std::size_t num_points,
                              const eigen_ops::Mat3& covariance,
                              const std::string& context) {
    std::lock_guard<std::mutex> lock(mutex_);

    CsvFile& f = fileFor(
        "covariance_matrices",
        "run_id,timestamp_ms,row_id,context,num_points,c00,c01,c02,c11,c12,c22"
    );

    const auto row_id = f.next_row_id.fetch_add(1);

    f.stream
        << run_id_ << ","
        << nowMs() << ","
        << row_id << ","
        << context << ","
        << num_points << ","
        << covariance.at(0,0) << ","
        << covariance.at(0,1) << ","
        << covariance.at(0,2) << ","
        << covariance.at(1,1) << ","
        << covariance.at(1,2) << ","
        << covariance.at(2,2)
        << "\n";

    f.stream.flush();
}


// ============================================================================
// PCA eigenvalues
// ============================================================================
void CsvLogger::logPcaEigenvalues(const eigen_ops::Vec3& eigenvalues,
                                  const std::string& context) {
    std::lock_guard<std::mutex> lock(mutex_);

    CsvFile& f = fileFor(
        "pca_eigenvalues",
        "run_id,timestamp_ms,row_id,context,lambda0,lambda1,lambda2"
    );

    const auto row_id = f.next_row_id.fetch_add(1);

    f.stream
        << run_id_ << ","
        << nowMs() << ","
        << row_id << ","
        << context << ","
        << eigenvalues.x << ","
        << eigenvalues.y << ","
        << eigenvalues.z
        << "\n";

    f.stream.flush();
}


// ============================================================================
// PCA eigenvectors
// ============================================================================
void CsvLogger::logPcaEigenvectors(const eigen_ops::Mat3& eigenvectors,
                                   const std::string& context) {
    std::lock_guard<std::mutex> lock(mutex_);

    CsvFile& f = fileFor(
        "pca_eigenvectors",
        "run_id,timestamp_ms,row_id,context,"
        "v0_x,v0_y,v0_z,v1_x,v1_y,v1_z,v2_x,v2_y,v2_z"
    );

    const auto row_id = f.next_row_id.fetch_add(1);

    f.stream
        << run_id_ << ","
        << nowMs() << ","
        << row_id << ","
        << context << ","
        << eigenvectors.at(0,0) << "," << eigenvectors.at(1,0) << "," << eigenvectors.at(2,0) << ","
        << eigenvectors.at(0,1) << "," << eigenvectors.at(1,1) << "," << eigenvectors.at(2,1) << ","
        << eigenvectors.at(0,2) << "," << eigenvectors.at(1,2) << "," << eigenvectors.at(2,2)
        << "\n";

    f.stream.flush();
}


// ============================================================================
// Bounding box
// ============================================================================
void CsvLogger::logBoundingBox(const eigen_ops::Vec3& center,
                               const eigen_ops::Mat3& axes,
                               const eigen_ops::Vec3& half_extents,
                               const std::string& context) {
    std::lock_guard<std::mutex> lock(mutex_);

    CsvFile& f = fileFor(
        "bounding_boxes",
        "run_id,timestamp_ms,row_id,context,"
        "center_x,center_y,center_z,"
        "axis0_x,axis0_y,axis0_z,axis1_x,axis1_y,axis1_z,axis2_x,axis2_y,axis2_z,"
        "half_extent_x,half_extent_y,half_extent_z"
    );

    const auto row_id = f.next_row_id.fetch_add(1);

    f.stream
        << run_id_ << ","
        << nowMs() << ","
        << row_id << ","
        << context << ","
        << center.x << "," << center.y << "," << center.z << ","
        << axes.at(0,0) << "," << axes.at(1,0) << "," << axes.at(2,0) << ","
        << axes.at(0,1) << "," << axes.at(1,1) << "," << axes.at(2,1) << ","
        << axes.at(0,2) << "," << axes.at(1,2) << "," << axes.at(2,2) << ","
        << half_extents.x << "," << half_extents.y << "," << half_extents.z
        << "\n";

    f.stream.flush();
}


// ============================================================================
// Cluster center
// ============================================================================
void CsvLogger::logClusterCenter(std::size_t num_points,
                                 const eigen_ops::Vec3& center,
                                 const std::string& context) {
    std::lock_guard<std::mutex> lock(mutex_);

    CsvFile& f = fileFor(
        "cluster_centers",
        "run_id,timestamp_ms,row_id,context,num_points,x,y,z"
    );

    const auto row_id = f.next_row_id.fetch_add(1);

    f.stream
        << run_id_ << ","
        << nowMs() << ","
        << row_id << ","
        << context << ","
        << num_points << ","
        << center.x << ","
        << center.y << ","
        << center.z
        << "\n";

    f.stream.flush();
}

} // namespace perception_sim
