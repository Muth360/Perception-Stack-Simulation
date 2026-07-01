#include "perception_sim/clustering.hpp"
#include "perception_sim/logger.hpp"
#include "perception_sim/timing.hpp"

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

namespace perception_sim {
namespace clustering {

ClusterVec extractClusters(const CloudConstPtr& input, const ClusterParams& params) {
    ScopedTimer timer("clustering.euclidean");

    ClusterVec clusters;
    if (input->empty()) {
        Logger::instance().warn("clustering", "extractClusters called on empty input cloud");
        return clusters;
    }

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(input);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(params.cluster_tolerance);
    ec.setMinClusterSize(params.min_cluster_size);
    ec.setMaxClusterSize(params.max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input);
    ec.extract(cluster_indices);

    clusters.reserve(cluster_indices.size());
    for (const auto& indices : cluster_indices) {
        CloudPtr cluster_cloud(new CloudT);
        cluster_cloud->points.reserve(indices.indices.size());
        for (int idx : indices.indices) {
            cluster_cloud->points.push_back(input->points[static_cast<std::size_t>(idx)]);
        }
        cluster_cloud->width = static_cast<uint32_t>(cluster_cloud->points.size());
        cluster_cloud->height = 1;
        cluster_cloud->is_dense = true;
        clusters.push_back(Cluster{cluster_cloud});
    }

    Logger::instance().info("clustering",
        LogFields().add("input_points", input->size())
                    .add("num_clusters", clusters.size())
                    .add("tolerance", params.cluster_tolerance)
                    .add("min_size", params.min_cluster_size)
                    .add("max_size", params.max_cluster_size).str());

    for (std::size_t i = 0; i < clusters.size(); ++i) {
        Logger::instance().debug("clustering",
            LogFields().add("cluster_index", i).add("size", clusters[i].points->size()).str());
    }

    return clusters;
}

} // namespace clustering
} // namespace perception_sim
