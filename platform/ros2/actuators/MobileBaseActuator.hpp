#pragma once

#include "platform/errors/Errors.hpp"
#include "platform/ros2/recording/McapRecorder.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <chrono>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <variant>

namespace platform::actuators {

template<typename HardwareInterface>
class MobileBaseActuator : public rclcpp::Node
{
public:
    MobileBaseActuator(std::shared_ptr<HardwareInterface> hardwareInterface, rclcpp::NodeOptions& options);

private:
    void velocityCallback(const geometry_msgs::msg::TwistStamped& msg);

    std::shared_ptr<HardwareInterface> hardwareInterface_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscriber_;
    std::string topicName_;
    int queueSize_ = 0;
};

template<typename HardwareInterface>
MobileBaseActuator<HardwareInterface>::MobileBaseActuator(std::shared_ptr<HardwareInterface> hardwareInterface,
                                                          rclcpp::NodeOptions& options)
    : Node("MobileBaseActuator", rclcpp::NodeOptions().allow_undeclared_parameters(false))
    , hardwareInterface_{std::move(hardwareInterface)}
{
    // Declare parameters with default values as needed
    this->declare_parameter<std::string>("topic_name", "/cmd_vel");
    this->declare_parameter<int>("queue_size", 10);

    // Now safely access the parameters
    topicName_ = this->get_parameter("topic_name").as_string();
    queueSize_ = this->get_parameter("queue_size").as_int();

    subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        topicName_, queueSize_, [this](const geometry_msgs::msg::TwistStamped& msg) {
            this->velocityCallback(msg);
        });
}

template<typename HardwareInterface>
void MobileBaseActuator<HardwareInterface>::velocityCallback(const geometry_msgs::msg::TwistStamped& msg)
{
    RCLCPP_INFO(
        this->get_logger(), "Received Twist: Linear X: '%.2f', Angular Z: '%.2f'", msg.twist.linear.x, msg.twist.angular.z);

    auto result = hardwareInterface_->setVelocity(msg.twist.linear.x, msg.twist.angular.z);
    if (std::holds_alternative<errors::MotorError>(result)) {
        RCLCPP_ERROR(this->get_logger(),
                     "Error occurred while setting velocity : %s",
                     errors::toString(std::get<errors::MotorError>(result)).c_str());
    }
}

} // namespace platform::actuators
