#pragma once

#include <fstream>
#include <mutex>
#include <sstream>
#include <string>

namespace perception_sim {

enum class LogLevel { Debug, Info, Warn, Error };

class Logger {
public:
    static Logger& instance();

    void setRunId(const std::string& run_id);
    void setLogFile(const std::string& path);

    void log(LogLevel level,
             const std::string& tag,
             const std::string& message);

    void debug(const std::string& tag, const std::string& msg) { log(LogLevel::Debug, tag, msg); }
    void info (const std::string& tag, const std::string& msg) { log(LogLevel::Info, tag, msg); }
    void warn (const std::string& tag, const std::string& msg) { log(LogLevel::Warn, tag, msg); }
    void error(const std::string& tag, const std::string& msg) { log(LogLevel::Error, tag, msg); }

private:
    Logger() = default;

    std::mutex mutex_;
    std::ofstream file_;
    bool file_open_ = false;

    std::string run_id_;
};

} // namespace perception_sim
