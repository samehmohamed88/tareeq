#pragma once

#include "platform/ros2/recording/McapRecorder.hpp"

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>

namespace platform::ros2::recording {

class McapRecorderNode : public rclcpp::Node
{
public:
    McapRecorderNode();

private:

    void velocityCallback(const geometry_msgs::msg::TwistStamped& msg)
    {
        RCLCPP_INFO(
            this->get_logger(), "Received Twist: Linear X: '%.2f', Angular Z: '%.2f'", msg.twist.linear.x, msg.twist.angular.z);
        std::cout << "received a twisted stamped message" << std::endl;
        recorder_.recordMessage("/cmd_vel", msg);
    }

    void imuCallback(const sensor_msgs::msg::Imu& msg, const std::string& topicName)
    {
        std::cout << "received an IMU message" << topicName << std::endl;
        recorder_.recordMessage(topicName, msg);
    }

    void odomFilteredCallback(const nav_msgs::msg::Odometry& msg)
    {
        std::cout << "received an Odometry message filtered with EKF" << std::endl;
        recorder_.recordMessage("/odom/filtered", msg);
    }

    McapRecorder recorder_;
//    using ApproximateTimePolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Imu, geometry_msgs::msg::TwistStamped>;
//    message_filters::Subscriber<sensor_msgs::msg::Imu> imuSubscription_;
//    message_filters::Subscriber<geometry_msgs::msg::TwistStamped> velocitySubscription_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu1Subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu2Subscription_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocitySubscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomFilteredSubscription_;
//    std::unique_ptr<message_filters::Synchronizer<ApproximateTimePolicy>> sync_;
};

} // namespace platform::ros2::recording
