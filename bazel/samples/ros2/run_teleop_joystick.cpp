#include "platform/ros2/teleop/JoystickNode.hpp"


#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <filesystem>
#include <cstdlib>
#include <memory>

int main(int argc, char* argv[]) {

    // Initialize ROS 2
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    // Configure node options with parameters
    options.allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true);

    // Create a multi-threaded executor
    rclcpp::executors::MultiThreadedExecutor executor;

    // Add both nodes to the executor
    executor.add_node(std::make_shared<platform::ros2::teleop::JoystickNode>(options));

    // Spin both nodes
    executor.spin();  // This will block until the nodes are shutdown

    // Shutdown ROS 2
    rclcpp::shutdown();

    return 0;
}