#include "platform/logging/LoggerFactory.hpp"
#include "platform/ros2/actuators/MobileBaseActuator.hpp"
#include "platform/vehicle/wave_rover/WaveRoverMotorController.hpp"
#include "platform/vehicle/wave_rover/WaveRoverNetworkDeviceManager.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace vehicles = platform::vehicle;
namespace waverover = platform::vehicle::waverover;
namespace logging = platform::logging;
namespace actuators = platform::actuators;

using DeviceManagerType = waverover::WaveRoverNetworkDeviceManager;
using MotorControllerType = waverover::WaveRoverMotorController<DeviceManagerType, logging::ConsoleLogger>;
using MobileBaseActuatorType = actuators::MobileBaseActuator<MotorControllerType>;

std::shared_ptr<actuators::MobileBaseActuator<MotorControllerType>> makeMobileBaseActuator()
{
    rclcpp::NodeOptions options;

    vehicles::VehicleConfig vehicleConfig_{0.0375, 0.13, .2};

    std::shared_ptr<logging::ConsoleLogger> logger_{logging::LoggerFactory::createLogger("console")};

    std::cout << "creating wave rover network device manager" << std::endl;
    std::shared_ptr<DeviceManagerType> deviceManager_ = std::make_shared<DeviceManagerType>();

    std::cout << "creating motor controller" << std::endl;
    std::shared_ptr<MotorControllerType> motorController_{
        std::make_shared<MotorControllerType>(deviceManager_, logger_, vehicleConfig_)};

    std::cout << "creating the node finally" << std::endl;
    return std::make_shared<MobileBaseActuatorType>(motorController_, options);
};

int main()
{
    // Initialize ROS 2
    rclcpp::init(0, nullptr);

    std::cout << " making mobile base actuator" << std::endl;

    rclcpp::spin(makeMobileBaseActuator());

    rclcpp::shutdown();

    return 0;
}
