#pragma once

#include "platform/logging/ROS2Logger.hpp"

#include <fmt/format.h>

#include <string>

namespace platform::logging {

class Logger
{
public:
    Logger(const std::string& name)
        : logger_{name}
    {}

    /// @brief Logs a debug message with formatting.
    /// @param format_string The format string (using { } as placeholders).
    /// @param args Arguments corresponding to placeholders in the format string.
    template<typename... Args>
    void logDebug(const std::string& format_string, Args&&... args) const
    {
        auto s = fmt::format(fmt::runtime(format_string), std::forward<Args>(args)...);
        logDebugImpl(s);
    }

    /// @brief Logs an informational message with formatting.
    /// @param format_string The format string.
    /// @param args Arguments to be formatted into the string.
    template<typename... Args>
    void logInfo(const std::string& format_string, Args&&... args)
    {
        auto s = fmt::format(fmt::runtime(format_string), std::forward<Args>(args)...);
        logInfoImpl(s);
    }

    /// @brief Logs a warning message with formatting.
    /// @param format_string The format string.
    /// @param args Arguments to be formatted into the string.
    template<typename... Args>
    void logWarn(const std::string& format_string, Args&&... args)
    {
        auto s = fmt::format(fmt::runtime(format_string), std::forward<Args>(args)...);
        logWarnImpl(s);
    }

    /// @brief Logs an error message with formatting.
    /// @param format_string The format string.
    /// @param args Arguments to be formatted into the string.
    template<typename... Args>
    void logError(const std::string& format_string, Args&&... args)
    {
        auto s = fmt::format(fmt::runtime(format_string), std::forward<Args>(args)...);
        logErrorImpl(s);
    }

private:
    /// @brief Logs a debug message using ROS2.
    /// @param message The formatted message to log.
    void logDebugImpl(const std::string& message) const { logger_.logDebugImpl(message); }

    /// @brief Logs an informational message using ROS2.
    /// @param message The formatted message to log.
    void logInfoImpl(const std::string& message) { logger_.logInfoImpl(message); }

    /// @brief Logs a warning message using ROS2.
    /// @param message The formatted message to log.
    void logWarnImpl(const std::string& message) { logger_.logWarnImpl(message); }

    /// @brief Logs an error message using ROS2.
    /// @param message The formatted message to log.
    void logErrorImpl(const std::string& message) { logger_.logErrorImpl(message); }

    ROS2Logger logger_;
};

} // namespace platform::logging
