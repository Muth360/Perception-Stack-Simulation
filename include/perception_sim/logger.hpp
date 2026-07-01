#pragma once
// -----------------------------------------------------------------------------
// logger.hpp
//
// Minimal structured logger used to record intermediate pipeline outputs
// (point counts, cluster stats, PCA results, bounding boxes, etc.) so that
// runs with a stock Eigen and a modified Eigen can be diffed easily.
// -----------------------------------------------------------------------------

#include <fstream>
#include <mutex>
#include <sstream>
#include <string>

namespace perception_sim {

enum class LogLevel { Debug, Info, Warn, Error };

class Logger {
public:
    // Returns the process-wide logger instance.
    static Logger& instance();

    // Optionally direct log output to a file in addition to stdout.
    // Pass an empty string to disable file logging.
    void setLogFile(const std::string& path);

    void log(LogLevel level, const std::string& tag, const std::string& message);

    // Convenience helpers.
    void debug(const std::string& tag, const std::string& message) { log(LogLevel::Debug, tag, message); }
    void info(const std::string& tag, const std::string& message)  { log(LogLevel::Info, tag, message); }
    void warn(const std::string& tag, const std::string& message)  { log(LogLevel::Warn, tag, message); }
    void error(const std::string& tag, const std::string& message) { log(LogLevel::Error, tag, message); }

private:
    Logger() = default;

    std::mutex mutex_;
    std::ofstream file_;
    bool file_open_ = false;
};

// Small helper for building "key=value, key=value" style log messages
// without every call site hand-rolling stringstream code.
class LogFields {
public:
    template <typename T>
    LogFields& add(const std::string& key, const T& value) {
        if (!first_) oss_ << ", ";
        oss_ << key << "=" << value;
        first_ = false;
        return *this;
    }

    std::string str() const { return oss_.str(); }

private:
    std::ostringstream oss_;
    bool first_ = true;
};

} // namespace perception_sim
