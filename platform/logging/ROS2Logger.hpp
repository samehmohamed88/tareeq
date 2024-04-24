#pragma once

#include <rclcpp/rclcpp.hpp>  // Required for ROS2 logging

#include <iostream>

namespace platform::logging {

/// @brief Logger implementation for ROS2.
class ROS2Logger  {
public:
    ROS2Logger(const rclcpp::Logger& logger)
        : logger_{logger}
    {}

    ROS2Logger(const std::string& name)
        : logger_{rclcpp::get_logger(name)}
    {}
    /// @brief Logs a debug message using ROS2.
    /// @param message The formatted message to log.
    void logDebugImpl(const std::string& message) const
    {
        RCLCPP_DEBUG(logger_, "%s", message.c_str());
    }

    /// @brief Logs an informational message using ROS2.
    /// @param message The formatted message to log.
    void logInfoImpl(const std::string& message)  {
        RCLCPP_INFO(logger_, "%s", message.c_str());
    }

    /// @brief Logs a warning message using ROS2.
    /// @param message The formatted message to log.
    void logWarnImpl(const std::string& message)  {
        RCLCPP_WARN(logger_, "%s", message.c_str());
    }

    /// @brief Logs an error message using ROS2.
    /// @param message The formatted message to log.
    void logErrorImpl(const std::string& message)  {
        RCLCPP_ERROR(logger_, "%s", message.c_str());
    }
private:
    rclcpp::Logger logger_;
};

} // namespace platform::logging
