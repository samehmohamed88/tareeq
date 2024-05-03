#include "platform/actuators/MobileBaseActuator.hpp"
#include "platform/io/BoostSerialDeviceManager.hpp"

#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <memory>


auto makeMobileBaseActuator()
    -> std::shared_ptr<platform::actuators::MobileBaseActuator>
{
    auto actuatorNodeOptions = rclcpp::NodeOptions().arguments({"--ros-args", "-r", "__node:=mobile_base_actuator"});
    return std::make_shared<platform::actuators::MobileBaseActuator>(
        "/cmd_vel",
        actuatorNodeOptions);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;

    auto actuatorNode = makeMobileBaseActuator();

    std::cout << "Adding actuatorNode to executor" << std::endl;
    exec.add_node(actuatorNode);
    std::cout << "actuatorNode added to executor" << std::endl;

    exec.spin();

    rclcpp::shutdown();
    return 0;
}
