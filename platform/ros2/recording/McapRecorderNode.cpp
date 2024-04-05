#include "platform/ros2/recording/McapRecorderNode.hpp"

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <memory>

namespace platform::ros2::recording {

McapRecorderNode::McapRecorderNode()
    : rclcpp::Node("McapRecorderNode")
{
    //    imuSubscription_.subscribe(this, "/imu_data");
    //    velocitySubscription_.subscribe(this, "/cmd_vel");

    // create topics in the recorder
    recorder_.createTopic("/zed_imu_data", "sensor_msgs/msg/Imu");
    recorder_.createTopic("/rover_imu_data", "sensor_msgs/msg/Imu");
    recorder_.createTopic("/cmd_vel", "geometry_msgs/msg/TwistStamped");
    recorder_.createTopic("/odom/filtered", "nav_msgs/msg/Odometry");

    velocitySubscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/cmd_vel", 10, [this](const geometry_msgs::msg::TwistStamped& msg) { this->velocityCallback(msg); });

    imu1Subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/zed_imu_data", 10, [this](const sensor_msgs::msg::Imu& msg) { this->imuCallback(msg, "/zed_imu_data"); });

    imu2Subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/rover_imu_data", 10, [this](const sensor_msgs::msg::Imu& msg) { this->imuCallback(msg, "/rover_imu_data"); });

    odomFilteredSubscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry& msg) { this->odomFilteredCallback(msg); });
}

} // namespace platform::ros2::recording

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<platform::ros2::recording::McapRecorderNode>());
    rclcpp::shutdown();
    return 0;
}
