// -----------------------------------------------------------------------------
// main.cpp
//
// Entry point: generates (or loads) a point cloud, runs the full pipeline,
// and prints timing statistics. See README / scripts/run_pipeline.sh for
// usage.
// -----------------------------------------------------------------------------
#include "perception_sim/pipeline.hpp"
#include "perception_sim/point_cloud_io.hpp"
#include "perception_sim/logger.hpp"
#include "perception_sim/timing.hpp"
#include "perception_sim/csv_logger.hpp"

#include <iostream>
#include <string>

int main(int argc, char** argv) {
    using namespace perception_sim;

    Logger::instance().setLogFile("data/perception_sim.log");
    Logger::instance().info("main", "perception_sim starting");

    // Initializes a fresh run_id and CSV output directory. Every
    // matrix multiplication, centroid, covariance matrix, PCA eigenvalue/
    // eigenvector, bounding box, and cluster center computed from this
    // point on is appended to data/csv/*.csv tagged with this run_id, so
    // separate runs (e.g. stock Eigen vs. a modified Eigen) can be
    // compared by filtering on run_id.
    CsvLogger::instance().beginRun("data/csv");
    Logger::instance().info("main", "csv instrumentation run_id=" + CsvLogger::instance().runId());

    // Optional: pass a .pcd/.ply path as argv[1] to use a real cloud instead
    // of the synthetic generator.
    CloudPtr input_cloud;
    if (argc > 1) {
        const std::string path = argv[1];
        input_cloud = io::loadPointCloud(path);
    } else {
        input_cloud = io::generateSyntheticCloud(/*num_objects=*/5,
                                                   /*points_per_object=*/250,
                                                   /*spread=*/10.0,
                                                   /*random_seed=*/42);
    }

    PipelineConfig config;
    config.output_image_path = "data/output_scene.png";

    PipelineResult result = runPipeline(input_cloud, config);

    std::cout << "\nDetected " << result.objects.size() << " object(s):\n";
    for (std::size_t i = 0; i < result.objects.size(); ++i) {
        const auto& obj = result.objects[i];
        std::cout << "  [" << i << "] points=" << obj.num_points
                  << " centroid=(" << obj.pca_result.centroid.x << ", "
                  << obj.pca_result.centroid.y << ", " << obj.pca_result.centroid.z << ")"
                  << " half_extents=(" << obj.box.half_extents.x << ", "
                  << obj.box.half_extents.y << ", " << obj.box.half_extents.z << ")\n";
    }

    TimingStats::instance().printSummary();
    return 0;
}
