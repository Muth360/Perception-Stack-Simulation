#include "perception_sim/bounding_box.hpp"
#include "perception_sim/logger.hpp"
#include "perception_sim/timing.hpp"
#include "perception_sim/csv_logger.hpp"

namespace perception_sim {
namespace bbox {

OrientedBoundingBox computeOrientedBoundingBox(const Cluster& cluster,
                                                const pca::ClusterPcaResult& pca_result,
                                                std::size_t cluster_index) {
    ScopedTimer timer("bbox.compute_oriented");
    const std::string context = "cluster_" + std::to_string(cluster_index);

    std::vector<eigen_ops::Vec3> pts;
    pts.reserve(cluster.points->size());
    for (const auto& p : cluster.points->points) {
        pts.push_back(eigen_ops::Vec3{p.x, p.y, p.z});
    }

    // Move points into the PCA (local) frame, then take an axis-aligned
    // extent there - this gives a tight oriented box in world space once
    // transformed back.
    const std::vector<eigen_ops::Vec3> local_pts =
        eigen_ops::transformToLocalFrame(pts, pca_result.centroid, pca_result.eigenvectors, context);
    const eigen_ops::Extent extent = eigen_ops::computeExtent(local_pts);

    const eigen_ops::Vec3 local_center{
        (extent.min.x + extent.max.x) * 0.5,
        (extent.min.y + extent.max.y) * 0.5,
        (extent.min.z + extent.max.z) * 0.5
    };
    const eigen_ops::Vec3 half_extents{
        (extent.max.x - extent.min.x) * 0.5,
        (extent.max.y - extent.min.y) * 0.5,
        (extent.max.z - extent.min.z) * 0.5
    };

    OrientedBoundingBox box;
    box.axes = pca_result.eigenvectors;
    box.half_extents = half_extents;
    box.center = eigen_ops::transformToWorldFrame(local_center, pca_result.centroid, pca_result.eigenvectors,
                                                   context + ":center");

    // Enumerate the 8 corners in the local frame (relative to local_center),
    // then transform each back to world space.
    int corner_idx = 0;
    for (double sx : {-1.0, 1.0}) {
        for (double sy : {-1.0, 1.0}) {
            for (double sz : {-1.0, 1.0}) {
                const eigen_ops::Vec3 local_corner{
                    local_center.x + sx * half_extents.x,
                    local_center.y + sy * half_extents.y,
                    local_center.z + sz * half_extents.z
                };
                box.corners[static_cast<std::size_t>(corner_idx++)] =
                    eigen_ops::transformToWorldFrame(local_corner, pca_result.centroid, pca_result.eigenvectors,
                                                      context + ":corner_" + std::to_string(corner_idx));
            }
        }
    }

    Logger::instance().debug("bbox",
        LogFields().add("center_x", box.center.x)
                    .add("center_y", box.center.y)
                    .add("center_z", box.center.z)
                    .add("half_extent_x", box.half_extents.x)
                    .add("half_extent_y", box.half_extents.y)
                    .add("half_extent_z", box.half_extents.z).str());

    CsvLogger::instance().logBoundingBox(box.center, box.axes, box.half_extents, context);

    return box;
}

} // namespace bbox
} // namespace perception_sim
