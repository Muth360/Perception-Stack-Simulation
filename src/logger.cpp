#include "perception_sim/logger.hpp"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <filesystem>

namespace perception_sim {

namespace {

std::string levelToString(LogLevel level) {
    switch (level) {
        case LogLevel::Debug: return "DEBUG";
        case LogLevel::Info:  return "INFO ";
        case LogLevel::Warn:  return "WARN ";
        case LogLevel::Error: return "ERROR";
    }
    return "?????";
}

std::string timestamp() {
    const auto now = std::chrono::system_clock::now();
    const auto t = std::chrono::system_clock::to_time_t(now);

    std::tm tm{};
    localtime_r(&t, &tm);  // thread-safe version

    std::ostringstream oss;
    oss << std::put_time(&tm, "%H:%M:%S");
    return oss.str();
}

} // namespace


Logger& Logger::instance() {
    static Logger logger;
    return logger;
}


// Set run ID (used for folder naming)
void Logger::setRunId(const std::string& run_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    run_id_ = run_id;
}


// Set log file (now run-aware)
void Logger::setLogFile(const std::string& base_path) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (file_.is_open()) {
        file_.close();
    }

    file_open_ = false;

    // If no run_id, fallback to base path
    std::string final_path = base_path;

    if (!run_id_.empty()) {
        std::string dir = "data/runs/" + run_id_;

        std::filesystem::create_directories(dir);

        final_path = dir + "/log.txt";
    }

    file_.open(final_path, std::ios::out | std::ios::trunc);
    file_open_ = file_.is_open();

    if (!file_open_) {
        std::cerr << "Failed to open log file: " << final_path << std::endl;
    }
}


// Main log function
void Logger::log(LogLevel level,
                 const std::string& tag,
                 const std::string& message) {
    std::lock_guard<std::mutex> lock(mutex_);

    std::ostringstream line;
    line << "[" << timestamp() << "] "
         << "[" << levelToString(level) << "] "
         << "[" << tag << "] "
         << message;

    std::cout << line.str() << std::endl;

    if (file_open_) {
        file_ << line.str() << std::endl;
        file_.flush();
    }
}

} // namespace perception_sim
