#pragma once

#include "platform/io/BoostSerialDeviceManager.hpp"
#include "platform/io/Status.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <utility>
#include <variant>

namespace platform::actuators {

class MobileBaseActuator : public rclcpp::Node
{
public:
    MobileBaseActuator(std::string topicName, const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions())
        : Node("MobileBaseActuator", node_options)
    {
        boostDeviceManager_.createDevice("/dev/ttyUSB0", 115200);
        // Subscribe to TwistStamped
//        subscriberStamped_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
//            topicName, queueSize_, [this](const geometry_msgs::msg::TwistStamped& msg) {
//                velocityCallbackStamped(msg);
//            });
        // Subscribe to Twist
        subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            topicName, queueSize_, [this](const geometry_msgs::msg::Twist& msg) {
                velocityCallback(msg);
            });
    };

private:
    void velocityCallbackStamped(const geometry_msgs::msg::TwistStamped& msg);

    void velocityCallback(const geometry_msgs::msg::Twist& msg) ;

    void processVelocity(double linear, double angular);

    int queueSize_ = 10;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscriberStamped_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
    platform::io::BoostSerialDeviceManager boostDeviceManager_;

    static constexpr double maxLinearSpeed_ = 0.5;
    static constexpr double maxAngularSpeed_ = 0.5;
    static constexpr double rearWheelRadius_ = 0.0375;
    static constexpr double rearWheelSeparation_ = 0.13;

    double normalizeToHalfRange(double value) {
        // Clamp the value to be between -1.0 and 1.0
        value = std::clamp(value, -1.0, 1.0);
        // Scale the clamped value to the range [-0.5, 0.5]
        return 0.01 * value;
    }

    std::string formatDouble(double number) {
        std::stringstream stream;
        // Apply fixed-point notation and set precision to 2 decimal places
        stream << std::fixed << std::setprecision(2) << number;
        return stream.str();
    }
};

} // namespace platform::actuators
