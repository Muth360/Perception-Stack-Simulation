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
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);

    std::tm tm{};
    localtime_r(&t, &tm);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%H:%M:%S");
    return oss.str();
}

} // namespace


Logger& Logger::instance() {
    static Logger instance;
    return instance;
}


void Logger::setRunId(const std::string& run_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    run_id_ = run_id;
}


void Logger::setLogFile(const std::string& base_path) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (file_.is_open()) file_.close();
    file_open_ = false;

    if (run_id_.empty()) {
        file_.open(base_path, std::ios::out | std::ios::trunc);
        file_open_ = file_.is_open();
        return;
    }

    std::string dir = "data/runs/" + run_id_;
    std::filesystem::create_directories(dir);

    std::string path = dir + "/log_" + run_id_ + ".txt";

    file_.open(path, std::ios::out | std::ios::trunc);
    file_open_ = file_.is_open();
}


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
