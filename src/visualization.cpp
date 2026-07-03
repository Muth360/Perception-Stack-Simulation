#include "perception_sim/visualization.hpp"
#include "perception_sim/logger.hpp"
#include "perception_sim/timing.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

namespace perception_sim {
namespace visualization {

namespace {

cv::Point worldToPixel(double wx, double wy, const RenderParams& params) {
    const double px = (wx - params.origin_x) / params.meters_per_pixel + params.image_width / 2.0;
    // Flip Y so that +y (world "forward/left") goes up in the image.
    const double py = params.image_height / 2.0 - (wy - params.origin_y) / params.meters_per_pixel;
    return cv::Point(static_cast<int>(std::lround(px)), static_cast<int>(std::lround(py)));
}

cv::Scalar clusterColor(std::size_t cluster_index) {
    // A small fixed palette, cycled by cluster index, so output is stable
    // and easy to diff visually between runs.
    static const std::vector<cv::Scalar> palette = {
        cv::Scalar(0, 165, 255),   // orange
        cv::Scalar(0, 255, 0),     // green
        cv::Scalar(255, 0, 0),     // blue
        cv::Scalar(0, 0, 255),     // red
        cv::Scalar(255, 255, 0),   // cyan
        cv::Scalar(255, 0, 255),   // magenta
        cv::Scalar(0, 255, 255),   // yellow
    };
    return palette[cluster_index % palette.size()];
}

} // namespace

cv::Mat renderScene(const CloudConstPtr& background_cloud,
                     const ClusterVec& clusters,
                     const std::vector<bbox::OrientedBoundingBox>& boxes,
                     const RenderParams& params) {
    ScopedTimer timer("visualization.render_scene");

    cv::Mat image(params.image_height, params.image_width, CV_8UC3, cv::Scalar(30, 30, 30));

    // Background points (e.g. pre-cluster / unfiltered noise) in dim gray.
    for (const auto& p : background_cloud->points) {
        const cv::Point pt = worldToPixel(p.x, p.y, params);
        if (pt.x >= 0 && pt.x < image.cols && pt.y >= 0 && pt.y < image.rows) {
            image.at<cv::Vec3b>(pt) = cv::Vec3b(80, 80, 80);
        }
    }

    // Cluster points, colored per-cluster.
    for (std::size_t ci = 0; ci < clusters.size(); ++ci) {
        const cv::Scalar color = clusterColor(ci);
        for (const auto& p : clusters[ci].points->points) {
            const cv::Point pt = worldToPixel(p.x, p.y, params);
            cv::circle(image, pt, 1, color, cv::FILLED);
        }
    }

    // Oriented bounding boxes: project onto the ground plane by taking the
    // 4 corners at the lower z-extent (indices 0,2,4,6 given the corner
    // enumeration in bounding_box.cpp: sx,sy vary while sz is fixed within
    // each pair) and draw the quadrilateral.
    for (std::size_t bi = 0; bi < boxes.size(); ++bi) {
        const auto& box = boxes[bi];
        // Corner order from computeOrientedBoundingBox: sx in {-,+}, sy in {-,+}, sz in {-,+}
        // index = sx*4 + sy*2 + sz (0/1 per sign). We want the bottom face (sz = -1 -> bit 0).
        const int idx_mm = 0; // sx=-1,sy=-1,sz=-1
        const int idx_mp = 2; // sx=-1,sy=+1,sz=-1
        const int idx_pp = 6; // sx=+1,sy=+1,sz=-1
        const int idx_pm = 4; // sx=+1,sy=-1,sz=-1

        std::vector<cv::Point> quad = {
            worldToPixel(box.corners[static_cast<std::size_t>(idx_mm)].x, box.corners[static_cast<std::size_t>(idx_mm)].y, params),
            worldToPixel(box.corners[static_cast<std::size_t>(idx_pm)].x, box.corners[static_cast<std::size_t>(idx_pm)].y, params),
            worldToPixel(box.corners[static_cast<std::size_t>(idx_pp)].x, box.corners[static_cast<std::size_t>(idx_pp)].y, params),
            worldToPixel(box.corners[static_cast<std::size_t>(idx_mp)].x, box.corners[static_cast<std::size_t>(idx_mp)].y, params),
        };

        const cv::Scalar color = clusterColor(bi);
        cv::polylines(image, quad, /*isClosed=*/true, color, 2, cv::LINE_AA);

        const cv::Point center_px = worldToPixel(box.center.x, box.center.y, params);
        cv::drawMarker(image, center_px, cv::Scalar(255, 255, 255), cv::MARKER_CROSS, 8, 1);

        cv::putText(image, "obj_" + std::to_string(bi), quad[0] + cv::Point(2, -2),
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv::LINE_AA);
    }

    Logger::instance().info("visualization",
        LogFields().add("num_clusters_drawn", clusters.size())
                    .add("num_boxes_drawn", boxes.size())
                    .add("image_width", params.image_width)
                    .add("image_height", params.image_height).str());

    return image;
}

void saveImage(const std::string& path, const cv::Mat& image) {
    cv::imwrite(path, image);
    Logger::instance().info("visualization", LogFields().add("action", "save_image").add("path", path).str());
}

} // namespace visualization
} // namespace perception_sim
