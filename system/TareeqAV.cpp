#include "platform/actuators/MobileBaseActuator.hpp"
#include "platform/io/BoostSerialDeviceManager.hpp"
#include "platform/slam/SLAMActionClient.hpp"

#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <memory>

//auto makeBoostSerialDeviceManager() -> std::shared_ptr<platform::io::BoostSerialDeviceManager>
//{
//    return std::make_shared<platform::io::BoostSerialDeviceManager>();
//}

auto makeMobileBaseActuator()
    -> std::shared_ptr<platform::actuators::MobileBaseActuator>
{
//    [boostSerialDeviceManager = boostSerialDeviceManager](double x, double z) {
//        std::string formatted_string =
//            std::string(R"({"T":13,"X":)" + std::to_string(x) + ",\"Z\":" + std::to_string(z) + "\n}", x, z);
//        boostSerialDeviceManager->writeToDevice("/dev/ttyUSB0", formatted_string);
//    },
//    boostSerialDeviceManager->createDevice("/dev/ttyUSB0", 115200);

    auto actuatorNodeOptions = rclcpp::NodeOptions().arguments({"--ros-args", "-r", "__node:=mobile_base_actuator"});
    return std::make_shared<platform::actuators::MobileBaseActuator>(
        "/cmd_vel",
        actuatorNodeOptions);
}

auto makeSLAMActionClient() -> std::shared_ptr<platform::slam::SLAMActionClient>
{
    auto slamActionClientOptions = rclcpp::NodeOptions().arguments({"--ros-args", "-r", "__node:=slam_action_client"});
    return std::make_shared<platform::slam::SLAMActionClient>(slamActionClientOptions);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;

   auto actuatorNode = makeMobileBaseActuator();
//    auto slamActionClient = makeSLAMActionClient();

   std::cout << "Adding actuatorNode to executor" << std::endl;
   exec.add_node(actuatorNode);
   std::cout << "actuatorNode added to executor" << std::endl;

//    std::cout << "Adding slamActionClient to executor" << std::endl;
//    exec.add_node(slamActionClient);
//    std::cout << "slamActionClient added to executor" << std::endl;

    exec.spin();

    rclcpp::shutdown();
    return 0;
}
