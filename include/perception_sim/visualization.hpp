#pragma once
// -----------------------------------------------------------------------------
// visualization.hpp
//
// Renders a top-down (bird's-eye) view of the point cloud, cluster points,
// and oriented bounding boxes using OpenCV. This is the final consumer of
// PCA/bounding-box output, so it's the last place a Eigen-driven change
// would visibly show up in the pipeline.
// -----------------------------------------------------------------------------

#include "perception_sim/types.hpp"
#include "perception_sim/bounding_box.hpp"
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace perception_sim {
namespace visualization {

struct RenderParams {
    int image_width = 800;
    int image_height = 800;
    double meters_per_pixel = 0.05; // world-to-pixel scale
    double origin_x = 0.0;          // world x mapped to image center
    double origin_y = 0.0;          // world y mapped to image center
};

// Renders the full scene: background cloud in gray, cluster points colored
// per-cluster, and oriented bounding boxes outlined in a contrasting color.
cv::Mat renderScene(const CloudConstPtr& background_cloud,
                     const ClusterVec& clusters,
                     const std::vector<bbox::OrientedBoundingBox>& boxes,
                     const RenderParams& params);

// Writes an OpenCV image to disk (PNG).
void saveImage(const std::string& path, const cv::Mat& image);

} // namespace visualization
} // namespace perception_sim
