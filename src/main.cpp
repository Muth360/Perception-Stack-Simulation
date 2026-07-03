#include "perception_sim/pipeline.hpp"
#include "perception_sim/point_cloud_io.hpp"
#include "perception_sim/logger.hpp"
#include "perception_sim/timing.hpp"
#include "perception_sim/csv_logger.hpp"

#include <iostream>
#include <string>
#include <filesystem>

int main(int argc, char** argv) {
    using namespace perception_sim;

    // ------------------------------------------------------------
    // 1. Start CSV run (single source of truth for run_id)
    // ------------------------------------------------------------
    CsvLogger::instance().beginRun("data/csv");
    std::string run_id = CsvLogger::instance().runId();

    // ------------------------------------------------------------
    // 2. Logger setup (same run_id)
    // ------------------------------------------------------------
    Logger::instance().setRunId(run_id);
    std::string log_path = run_dir + "/log_" + run_id + ".txt";
    Logger::instance().setLogFile(log_path);

    Logger::instance().info("main", "run_id=" + run_id);

    // ------------------------------------------------------------
    // 3. Input cloud
    // ------------------------------------------------------------
    CloudPtr input_cloud;

    if (argc > 1) {
        input_cloud = io::loadPointCloud(argv[1]);
    } else {
        input_cloud = io::generateSyntheticCloud(5, 250, 10.0,
    static_cast<unsigned>(std::time(nullptr)));
    }

    // ------------------------------------------------------------
    // 4. Output paths (ALL per-run)
    // ------------------------------------------------------------
    std::string run_dir = "data/runs/" + run_id;
    std::filesystem::create_directories(run_dir);

    PipelineConfig config;
    config.output_image_path =
        run_dir + "/output_scene_" + run_id + ".png";

    // ------------------------------------------------------------
    // 5. Run pipeline
    // ------------------------------------------------------------
    PipelineResult result = runPipeline(input_cloud, config);

    // ------------------------------------------------------------
    // 6. Print results
    // ------------------------------------------------------------
    std::cout << "\nDetected " << result.objects.size() << " object(s)\n";

    for (size_t i = 0; i < result.objects.size(); i++) {
        const auto& obj = result.objects[i];

        std::cout << "[" << i << "] points=" << obj.num_points
                  << " centroid=("
                  << obj.pca_result.centroid.x << ", "
                  << obj.pca_result.centroid.y << ", "
                  << obj.pca_result.centroid.z << ")\n";
    }

    TimingStats::instance().printSummary();
    return 0;
}
