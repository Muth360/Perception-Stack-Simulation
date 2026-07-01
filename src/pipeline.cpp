#include "perception_sim/pipeline.hpp"
#include "perception_sim/logger.hpp"
#include "perception_sim/timing.hpp"

namespace perception_sim {

PipelineResult runPipeline(const CloudPtr& input_cloud, const PipelineConfig& config) {
    ScopedTimer total_timer("pipeline.total");
    auto& log = Logger::instance();

    log.info("pipeline", LogFields().add("stage", "start").add("input_points", input_cloud->size()).str());

    // 1) PCL filtering: passthrough then voxel downsample.
    CloudPtr after_passthrough = filtering::passThroughFilter(input_cloud, config.passthrough);
    CloudPtr after_voxel = filtering::voxelFilter(after_passthrough, config.voxel);

    // 2) Euclidean clustering.
    ClusterVec clusters = clustering::extractClusters(after_voxel, config.cluster);

    // 3) Eigen matrix operations: PCA + oriented bounding box per cluster.
    std::vector<DetectedObject> objects;
    objects.reserve(clusters.size());
    for (std::size_t i = 0; i < clusters.size(); ++i) {
        const auto& cluster = clusters[i];
        pca::ClusterPcaResult pca_result = pca::analyzeCluster(cluster);
        bbox::OrientedBoundingBox box = bbox::computeOrientedBoundingBox(cluster, pca_result);

        DetectedObject obj;
        obj.pca_result = pca_result;
        obj.box = box;
        obj.num_points = cluster.points->size();
        objects.push_back(obj);

        log.info("pipeline",
            LogFields().add("stage", "object_detected")
                        .add("object_index", i)
                        .add("num_points", obj.num_points)
                        .add("centroid_x", obj.pca_result.centroid.x)
                        .add("centroid_y", obj.pca_result.centroid.y)
                        .add("centroid_z", obj.pca_result.centroid.z)
                        .add("half_extent_x", obj.box.half_extents.x)
                        .add("half_extent_y", obj.box.half_extents.y)
                        .add("half_extent_z", obj.box.half_extents.z).str());
    }

    // 4) OpenCV visualization.
    std::vector<bbox::OrientedBoundingBox> boxes;
    boxes.reserve(objects.size());
    for (const auto& obj : objects) boxes.push_back(obj.box);

    cv::Mat scene = visualization::renderScene(after_voxel, clusters, boxes, config.render);
    visualization::saveImage(config.output_image_path, scene);

    log.info("pipeline",
        LogFields().add("stage", "complete")
                    .add("num_objects", objects.size())
                    .add("output_image", config.output_image_path).str());

    PipelineResult result;
    result.filtered_cloud = after_voxel;
    result.clusters = std::move(clusters);
    result.objects = std::move(objects);
    return result;
}

} // namespace perception_sim
