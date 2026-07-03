#include "perception_sim/pipeline.hpp"
#include "perception_sim/point_cloud_io.hpp"
#include "perception_sim/logger.hpp"
#include "perception_sim/timing.hpp"
#include "perception_sim/csv_logger.hpp"

#include <iostream>
#include <string>
#include <filesystem>
#include <ctime>

int main(int argc, char** argv) {
    using namespace perception_sim;

    // ------------------------------------------------------------
    // 1. Start CSV run (generates run_id)
    // ------------------------------------------------------------
    CsvLogger::instance().beginRun("data/csv");
    std::string run_id = CsvLogger::instance().runId();

    // ------------------------------------------------------------
    // 2. Per-run directory
    // ------------------------------------------------------------
    std::string run_dir = "data/runs/" + run_id;
    std::filesystem::create_directories(run_dir);

    // ------------------------------------------------------------
    // 3. Logger setup
    // ------------------------------------------------------------
    Logger::instance().setRunId(run_id);

    std::string log_path = run_dir + "/log_" + run_id + ".txt";
    Logger::instance().setLogFile(log_path);

    Logger::instance().info("main", "run_id=" + run_id);

    // ------------------------------------------------------------
    // 4. Seed control (IMPORTANT CHANGE)
    // ------------------------------------------------------------
    static unsigned seed = 0;

    if (argc > 2) {
        seed = static_cast<unsigned>(std::stoi(argv[2]));
    } else {
        seed = 42;
    }

    Logger::instance().info("main", "seed=" + std::to_string(seed));

    // ------------------------------------------------------------
    // 5. Input cloud generation
    // ------------------------------------------------------------
    CloudPtr input_cloud;

    if (argc > 1) {
        input_cloud = io::loadPointCloud(argv[1]);
    } else {
        input_cloud = io::generateSyntheticCloud(
            5,
            250,
            10.0,
            seed
        );
    }

    // ------------------------------------------------------------
    // 6. Pipeline config
    // ------------------------------------------------------------
    PipelineConfig config;
    config.output_image_path = run_dir + "/output_scene.png";

    // ------------------------------------------------------------
    // 7. Run pipeline
    // ------------------------------------------------------------
    PipelineResult result = runPipeline(input_cloud, config);

    // ------------------------------------------------------------
    // 8. Print results
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

    // ------------------------------------------------------------
    // 9. Timing summary
    // ------------------------------------------------------------
    TimingStats::instance().printSummary();

    return 0;
}
