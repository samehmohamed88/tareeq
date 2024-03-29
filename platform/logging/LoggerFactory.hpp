#pragma once

#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <string>

namespace platform::devices {

class ILogger
{
public:
    virtual ~ILogger() = default;
    virtual void logInfo(const std::string& message) = 0;
    virtual void logError(const std::string& message) = 0;
    // Add more log levels as needed
};

class ConsoleLogger : public ILogger
{
public:
    void logInfo(const std::string& message) override { std::cout << "Info: " << message << std::endl; }

    void logError(const std::string& message) override { std::cerr << "Error: " << message << std::endl; }
};

class Ros2Logger : public ILogger
{
public:
    void logInfo(const std::string& message) override
    {
        RCLCPP_INFO(rclcpp::get_logger("MotorController"), "%s", message.c_str());
    }

    void logError(const std::string& message) override
    {
        RCLCPP_ERROR(rclcpp::get_logger("MotorController"), "%s", message.c_str());
    }
};

class LoggerFactory
{
public:
    // Factory method
    static std::unique_ptr<ILogger> createLogger(const std::string& loggerType, const std::string& loggerName = "")
    {
        if (loggerType == "ROS2") {
            return std::make_unique<Ros2Logger>(loggerName);
        } else {
            // Default to console logger if no specific type is requested or recognized
            return std::make_unique<ConsoleLogger>();
        }
    }
};

} // namespace platform::devices
