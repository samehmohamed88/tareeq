#pragma once

#include "platform/io/Status.hpp"

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <variant>

namespace platform::actuators {

class MobileBaseActuator : public rclcpp::Node
{
public:
    MobileBaseActuator(rclcpp::NodeOptions& options, std::string  topicName, std::function<io::Status(double, double)>& hardwareInterface)
        : Node("MobileBaseActuator", options)
        , topicName_{std::move(topicName)}
        , hardwareInterface_{std::move(hardwareInterface)}
    {
        subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            topicName_, queueSize_, [this](const geometry_msgs::msg::TwistStamped& msg) {
                velocityCallback(msg);
            });
    };

private:
    void velocityCallback(const geometry_msgs::msg::TwistStamped& msg);
    int queueSize_ = 0;
    std::string topicName_;
    std::function<io::Status(double, double)> hardwareInterface_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscriber_;

};

} // namespace platform::actuators
