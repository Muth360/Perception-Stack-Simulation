#include "perception_sim/timing.hpp"
#include "perception_sim/logger.hpp"

#include <algorithm>
#include <numeric>
#include <sstream>

namespace perception_sim {

TimingStats& TimingStats::instance() {
    static TimingStats stats;
    return stats;
}

void TimingStats::record(const std::string& stage_name, double milliseconds) {
    std::lock_guard<std::mutex> lock(mutex_);
    stats_[stage_name].samples_ms.push_back(milliseconds);
}

void TimingStats::printSummary() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::ostringstream oss;
    oss << "\n=== Timing summary (ms) ===\n";
    for (const auto& [name, stat] : stats_) {
        if (stat.samples_ms.empty()) continue;
        const double total = std::accumulate(stat.samples_ms.begin(), stat.samples_ms.end(), 0.0);
        const double mean = total / static_cast<double>(stat.samples_ms.size());
        const double mn = *std::min_element(stat.samples_ms.begin(), stat.samples_ms.end());
        const double mx = *std::max_element(stat.samples_ms.begin(), stat.samples_ms.end());
        oss << "  " << name
            << ": count=" << stat.samples_ms.size()
            << " total=" << total
            << " mean=" << mean
            << " min=" << mn
            << " max=" << mx << "\n";
    }
    Logger::instance().info("timing", oss.str());
}

void TimingStats::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    stats_.clear();
}

ScopedTimer::ScopedTimer(std::string stage_name)
    : stage_name_(std::move(stage_name)), start_(std::chrono::high_resolution_clock::now()) {}

ScopedTimer::~ScopedTimer() {
    const auto end = std::chrono::high_resolution_clock::now();
    const double ms = std::chrono::duration<double, std::milli>(end - start_).count();
    TimingStats::instance().record(stage_name_, ms);
}

} // namespace perception_sim
