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
    // 1. CSV run system
    // ------------------------------------------------------------
    CsvLogger::instance().beginRun("data/csv");
    std::string run_id = CsvLogger::instance().runId();

    // ------------------------------------------------------------
    // 2. Logger setup
    // ------------------------------------------------------------
    Logger::instance().setRunId(run_id);
    Logger::instance().setLogFile("data/log.txt");

    Logger::instance().info("main", "run_id=" + run_id);

    // ------------------------------------------------------------
    // 3. Seed handling (IMPORTANT FIX)
    // ------------------------------------------------------------
    static unsigned seed = 42;
    if (argc > 2) seed = static_cast<unsigned>(std::stoi(argv[2]));

    Logger::instance().info("main", "seed=" + std::to_string(seed));

    // ------------------------------------------------------------
    // 4. Input cloud
    // ------------------------------------------------------------
    CloudPtr input_cloud;

    // IMPORTANT FIX: "synthetic" keyword means generate data
    if (argc > 1 && std::string(argv[1]) != "synthetic") {
        input_cloud = io::loadPointCloud(argv[1]);
    } else {
        input_cloud = io::generateSyntheticCloud(
            5, 250, 10.0, seed
        );
    }

    // ------------------------------------------------------------
    // 5. Run folder output (per run)
    // ------------------------------------------------------------
    std::string run_dir = "data/runs/" + run_id;
    std::filesystem::create_directories(run_dir);

    PipelineConfig config;
    config.output_image_path =
        run_dir + "/output_scene_" + run_id + ".png";

    // ------------------------------------------------------------
    // 6. Run pipeline
    // ------------------------------------------------------------
    PipelineResult result = runPipeline(input_cloud, config);

    // ------------------------------------------------------------
    // 7. Print results
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
