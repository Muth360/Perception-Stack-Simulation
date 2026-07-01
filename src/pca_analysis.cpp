#include "perception_sim/pca_analysis.hpp"
#include "perception_sim/logger.hpp"
#include "perception_sim/timing.hpp"

namespace perception_sim {
namespace pca {

ClusterPcaResult analyzeCluster(const Cluster& cluster) {
    ScopedTimer timer("pca.analyze_cluster");

    std::vector<eigen_ops::Vec3> pts;
    pts.reserve(cluster.points->size());
    for (const auto& p : cluster.points->points) {
        pts.push_back(eigen_ops::Vec3{p.x, p.y, p.z});
    }

    const eigen_ops::PcaResult r = eigen_ops::computePca(pts);

    ClusterPcaResult out;
    out.centroid = r.mean;
    out.eigenvalues = r.eigenvalues;
    out.eigenvectors = r.eigenvectors;

    Logger::instance().debug("pca",
        LogFields().add("cluster_size", pts.size())
                    .add("centroid_x", out.centroid.x)
                    .add("centroid_y", out.centroid.y)
                    .add("centroid_z", out.centroid.z)
                    .add("eigval_0", out.eigenvalues.x)
                    .add("eigval_1", out.eigenvalues.y)
                    .add("eigval_2", out.eigenvalues.z).str());

    return out;
}

} // namespace pca
} // namespace perception_sim
