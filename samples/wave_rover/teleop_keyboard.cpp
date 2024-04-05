#include "platform/ros2/teleop/KeyboardTeleop.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main() {
    // Initialize ROS 2
    rclcpp::init(0, nullptr);

    rclcpp::spin(std::make_shared<platform::ros2::teleop::KeyboardTeleop>());

    rclcpp::shutdown();

    return 0;
}
