#include "perception_sim/logger.hpp"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>

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
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&t), "%H:%M:%S");
    return oss.str();
}
} // namespace

Logger& Logger::instance() {
    static Logger logger;
    return logger;
}

void Logger::setLogFile(const std::string& path) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (file_.is_open()) file_.close();
    file_open_ = false;
    if (!path.empty()) {
        file_.open(path, std::ios::out | std::ios::trunc);
        file_open_ = file_.is_open();
    }
}

void Logger::log(LogLevel level, const std::string& tag, const std::string& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::ostringstream line;
    line << "[" << timestamp() << "] [" << levelToString(level) << "] [" << tag << "] " << message;

    std::cout << line.str() << std::endl;
    if (file_open_) {
        file_ << line.str() << std::endl;
        file_.flush();
    }
}

} // namespace perception_sim
