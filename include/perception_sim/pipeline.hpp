#pragma once
// -----------------------------------------------------------------------------
// pipeline.hpp
//
// Wires together the full stack:
//   Point Cloud -> PCL Filtering -> Euclidean Clustering ->
//   Eigen Matrix Operations (PCA + OBB) -> OpenCV Visualization -> Detected Objects
//
// Every stage logs its intermediate output and records its timing, so a run
// with a stock Eigen can be diffed against a run with a modified Eigen.
// -----------------------------------------------------------------------------

#include "perception_sim/types.hpp"
#include "perception_sim/filtering.hpp"
#include "perception_sim/clustering.hpp"
#include "perception_sim/pca_analysis.hpp"
#include "perception_sim/bounding_box.hpp"
#include "perception_sim/visualization.hpp"

#include <string>
#include <vector>

namespace perception_sim {

struct PipelineConfig {
    filtering::VoxelFilterParams voxel;
    filtering::PassThroughParams passthrough;
    clustering::ClusterParams cluster;
    visualization::RenderParams render;

    std::string output_image_path = "data/output_scene.png";
    std::string log_file_path = "data/perception_sim.log";
};

struct DetectedObject {
    pca::ClusterPcaResult pca_result;
    bbox::OrientedBoundingBox box;
    std::size_t num_points = 0;
};

struct PipelineResult {
    CloudPtr filtered_cloud;
    ClusterVec clusters;
    std::vector<DetectedObject> objects;
};

// Runs the full pipeline on `input_cloud` using `config`. Returns everything
// downstream code (or tests) might want to inspect.
PipelineResult runPipeline(const CloudPtr& input_cloud, const PipelineConfig& config);

} // namespace perception_sim
