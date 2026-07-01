#pragma once
// -----------------------------------------------------------------------------
// timing.hpp
//
// Lightweight stage timer. Each pipeline stage is wrapped in a ScopedTimer so
// we can compare wall-clock cost before/after swapping in a modified Eigen
// (a heavier or slower Eigen build should show up here).
// -----------------------------------------------------------------------------

#include <chrono>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace perception_sim {

class TimingStats {
public:
    static TimingStats& instance();

    void record(const std::string& stage_name, double milliseconds);

    // Prints a summary table (count, total, mean, min, max) per stage.
    void printSummary() const;

    void reset();

private:
    struct Stat {
        std::vector<double> samples_ms;
    };

    mutable std::mutex mutex_;
    std::map<std::string, Stat> stats_;
};

// RAII helper: construct at the top of a scope, timing stops and is recorded
// automatically when the object goes out of scope.
class ScopedTimer {
public:
    explicit ScopedTimer(std::string stage_name);
    ~ScopedTimer();

    ScopedTimer(const ScopedTimer&) = delete;
    ScopedTimer& operator=(const ScopedTimer&) = delete;

private:
    std::string stage_name_;
    std::chrono::high_resolution_clock::time_point start_;
};

} // namespace perception_sim
