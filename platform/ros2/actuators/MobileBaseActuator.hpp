#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <memory>
#include <chrono>
#include <random>
#include <string>
#include <utility>

namespace platform::actuators {

template <typename HardwareInterface>
class MobileBaseActuator : public rclcpp::Node {
public:
    MobileBaseActuator(std::unique_ptr<HardwareInterface> hardwareInterface, rclcpp::NodeOptions& options);
private:

    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    std::unique_ptr<HardwareInterface> hardwareInterface_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
    std::string topic_name_;
    int queue_size_ = 0;
};

template<typename HardwareInterface>
MobileBaseActuator<HardwareInterface>::MobileBaseActuator(std::unique_ptr<HardwareInterface> hardwareInterface, rclcpp::NodeOptions& options)
    : hardwareInterface_{std::move(hardwareInterface)}
{
    // Declare parameters with default values as needed
    this->declare_parameter<std::string>("topic_name", "default_vel_topic");
    this->declare_parameter<int>("queue_size", 10);

    // Now safely access the parameters
    topic_name_ = this->get_parameter("topic_name").as_string();
    queue_size_ = this->get_parameter("publish_rate_ms").as_int();

    subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        topic_name_,
        queue_size_,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) { this->twist_callback(msg); });
}

template<typename HardwareInterface>
void MobileBaseActuator<HardwareInterface>::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{}


} // namespace platform::actuators
