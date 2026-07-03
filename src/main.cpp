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

    // ============================================================
    // 1. Start CSV run FIRST (this generates run_id)
    // ============================================================
    CsvLogger::instance().beginRun("data/csv");

    std::string run_id = CsvLogger::instance().runId();

    // ============================================================
    // 2. Initialize Logger with SAME run_id
    // ============================================================
    Logger::instance().setRunId(run_id);
    Logger::instance().setLogFile("data/log.txt");

    Logger::instance().log(LogLevel::Info, "main",
        "perception_sim starting run_id=" + run_id);

    Logger::instance().log(LogLevel::Info, "main",
        "csv instrumentation run_id=" + run_id);

    // ============================================================
    // 3. Input cloud
    // ============================================================
    CloudPtr input_cloud;

    if (argc > 1) {
        input_cloud = io::loadPointCloud(argv[1]);
    } else {
        input_cloud = io::generateSyntheticCloud(
            5, 250, 10.0, 42
        );
    }

    // ============================================================
    // 4. Pipeline config (you can extend later)
    // ============================================================
    PipelineConfig config;

    // IMPORTANT: now per-run output (optional but recommended)
    config.output_image_path =
        "data/runs/" + run_id + "/output_scene.png";

    // ============================================================
    // 5. Run pipeline
    // ============================================================
    PipelineResult result = runPipeline(input_cloud, config);

    // ============================================================
    // 6. Print results
    // ============================================================
    std::cout << "\nDetected " << result.objects.size() << " object(s):\n";

    for (std::size_t i = 0; i < result.objects.size(); ++i) {
        const auto& obj = result.objects[i];

        std::cout << "  [" << i << "] points=" << obj.num_points
                  << " centroid=("
                  << obj.pca_result.centroid.x << ", "
                  << obj.pca_result.centroid.y << ", "
                  << obj.pca_result.centroid.z << ")"
                  << " half_extents=("
                  << obj.box.half_extents.x << ", "
                  << obj.box.half_extents.y << ", "
                  << obj.box.half_extents.z << ")\n";
    }

    // ============================================================
    // 7. Timing
    // ============================================================
    TimingStats::instance().printSummary();

    return 0;
}
