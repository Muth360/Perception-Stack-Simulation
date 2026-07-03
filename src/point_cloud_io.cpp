#include "perception_sim/point_cloud_io.hpp"
#include "perception_sim/logger.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <random>
#include <stdexcept>

namespace perception_sim {
namespace io {

CloudPtr loadPointCloud(const std::string& path) {
    CloudPtr cloud(new CloudT);

    const bool is_ply = path.size() > 4 && path.substr(path.size() - 4) == ".ply";
    int rc = -1;
    if (is_ply) {
        rc = pcl::io::loadPLYFile<PointT>(path, *cloud);
    } else {
        rc = pcl::io::loadPCDFile<PointT>(path, *cloud);
    }

    if (rc != 0) {
        throw std::runtime_error("Failed to load point cloud from: " + path);
    }

    Logger::instance().info("point_cloud_io",
        LogFields().add("action", "load").add("path", path).add("points", cloud->size()).str());
    return cloud;
}

CloudPtr generateSyntheticCloud(int num_objects,
                                 int points_per_object,
                                 double spread,
                                 unsigned int random_seed) {
    CloudPtr cloud(new CloudT);
    std::mt19937 rng(random_seed);
    std::uniform_real_distribution<double> center_dist(-spread, spread);
    std::normal_distribution<double> jitter(0.0, 0.35);
    std::uniform_real_distribution<double> height_dist(0.2, 1.8);

    for (int obj = 0; obj < num_objects; ++obj) {
        const double cx = center_dist(rng);
        const double cy = center_dist(rng);
        const double cz = height_dist(rng) * 0.5;

        for (int i = 0; i < points_per_object; ++i) {
            PointT p;
            p.x = static_cast<float>(cx + jitter(rng));
            p.y = static_cast<float>(cy + jitter(rng));
            p.z = static_cast<float>(cz + jitter(rng) * 0.5);
            cloud->push_back(p);
        }
    }

    // A light scattering of background noise so filtering stages have
    // something to remove.
    std::uniform_real_distribution<double> noise_dist(-spread * 1.5, spread * 1.5);
    const int num_noise = num_objects * points_per_object / 4;
    for (int i = 0; i < num_noise; ++i) {
        PointT p;
        p.x = static_cast<float>(noise_dist(rng));
        p.y = static_cast<float>(noise_dist(rng));
        p.z = static_cast<float>(std::abs(noise_dist(rng)) * 0.05);
        cloud->push_back(p);
    }

    cloud->width = static_cast<uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = true;

    Logger::instance().info("point_cloud_io",
        LogFields().add("action", "generate_synthetic")
                    .add("num_objects", num_objects)
                    .add("points_per_object", points_per_object)
                    .add("total_points", cloud->size())
                    .add("seed", random_seed).str());
    return cloud;
}

void savePointCloud(const std::string& path, const CloudT& cloud) {
    pcl::io::savePCDFileASCII(path, cloud);
    Logger::instance().info("point_cloud_io",
        LogFields().add("action", "save").add("path", path).add("points", cloud.size()).str());
}

} // namespace io
} // namespace perception_sim
