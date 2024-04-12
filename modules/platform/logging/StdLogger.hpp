#pragma once

#include "platform/logging/LoggerInterface.hpp"

#include <iostream>

namespace platform::logging {

/// @brief Standard logger implementation using std::cout.
class StdLogger : public LoggerInterface
{
protected:
    /// @brief Logs a debug message using std::cout.
    /// @param message The formatted message to log.
    void logDebugImpl(const std::string& message) override { std::cout << "Debug: " << message << std::endl; }

    /// @brief Logs an informational message using std::cout.
    /// @param message The formatted message to log.
    void logInfoImpl(const std::string& message) override { std::cout << "Info: " << message << std::endl; }

    /// @brief Logs a warning message using std::cout.
    /// @param message The formatted message to log.
    void logWarnImpl(const std::string& message) override { std::cout << "Warn: " << message << std::endl; }

    /// @brief Logs an error message using std::cout.
    /// @param message The formatted message to log.
    void logErrorImpl(const std::string& message) override { std::cout << "Error: " << message << std::endl; }
};

} // namespace platform::logging
