#pragma once

#include "platform/actuators/SabertoothMotorController.hpp"
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

    ~MobileBaseActuator() {
        stopMotors();
    }

private:
//    void velocityCallbackStamped(const geometry_msgs::msg::TwistStamped& msg);

    void stopMotors() {
        sabertoothMotorController_.mixedModeDrive(64, 64); // Stop both motors
    }

    void velocityCallback(const geometry_msgs::msg::Twist& msg) ;

    void processVelocity(double linear, double angular);

    int queueSize_ = 10;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscriberStamped_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
    SabertoothMotorController sabertoothMotorController_;


    static constexpr double maxLinearSpeed_ = 1.0;
    static constexpr double maxAngularSpeed_ = 1.0;
    static constexpr double rearWheelRadius_ = 0.0375;
    static constexpr double rearWheelSeparation_ = 0.13;

    double normalizeToHalfRange(double value) {
        // Clamp the value to be between -1.0 and 1.0
        value = std::clamp(value, -1.0, 1.0);
        // Scale the clamped value to the range [-0.5, 0.5]
        return 0.01 * value;
    }

};

} // namespace platform::actuators
