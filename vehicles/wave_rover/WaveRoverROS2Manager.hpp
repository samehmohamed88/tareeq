#pragma once

#include "platform/io/Status.hpp"
#include "platform/logging/LoggerFactory.hpp"
#include "platform/ros2/actuators/MobileBaseActuator.hpp"
#include "platform/ros2/localization/IMUPublisher.hpp"
#include "platform/ros2/localization/StateEstimationNode.hpp"
#include "platform/sensors/imu/IMUData.hpp"
#include "platform/vehicle/wave_rover/WaveRoverIMUController.hpp"
#include "platform/vehicle/wave_rover/WaveRoverMotorController.hpp"
#include "platform/vehicle/wave_rover/WaveRoverNetworkDeviceManager.hpp"
#include "platform/vehicle/wave_rover/WaveRoverUtils.hpp"
#include "platform/zed/ZEDCamera.hpp"
#include "platform/zed/ZedIMUController.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <utility>

namespace platform::vehicle::waverover {

namespace vehicles = platform::vehicle;
namespace waverover = platform::vehicle::waverover;
namespace logging = platform::logging;
namespace imu = platform::sensors::imu;
namespace actuators = platform::actuators;
namespace localization = platform::ros2::localization;

using DeviceManagerType = waverover::WaveRoverNetworkDeviceManager;
using MotorControllerType = waverover::WaveRoverMotorController<DeviceManagerType, logging::ConsoleLogger>;
using IMU1ControllerType = platform::zed::ZedIMUController<imu::IMUData>;
using IMU2ControllerType = waverover::WaveRoverIMUController<imu::IMUData>;
using IMUPublisherType = localization::IMUPublisher<IMU1ControllerType, IMU2ControllerType>;

class WaveRoverROS2Manager
{

public:
    WaveRoverROS2Manager();

    void run();

private:
    std::shared_ptr<IMUPublisherType> makeIMUPublisher()
    {
        rclcpp::NodeOptions options;
        // Configure node options with parameters
        options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);

        // Mock or set parameters as required by the node
        options.parameter_overrides(
            {{"topic_name", "/imu_data"}, {"imu_frame_id", "imu_link"}, {"publish_rate_ms", 10000}});

        return std::make_shared<IMUPublisherType>(options, imu1Controller_, imu2Controller_);
    };

    std::shared_ptr<actuators::MobileBaseActuator<MotorControllerType>> makeMobileBaseActuator()
    {
        rclcpp::NodeOptions options;
        // Configure node options with parameters
        options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);

        // Mock or set parameters as required by the node
        options.parameter_overrides({{"topic_name", "/cmd_vel"}, {"queue_size", 10}});

        return std::make_shared<actuators::MobileBaseActuator<MotorControllerType>>(motorController_, options);
    };

    vehicles::VehicleConfig vehicleConfig_;
    std::shared_ptr<logging::ConsoleLogger> logger_;
    std::shared_ptr<DeviceManagerType> deviceManager_;
    std::shared_ptr<MotorControllerType> motorController_;
    std::shared_ptr<platform::zed::ZEDCamera> zedCamera_;
    std::shared_ptr<IMU1ControllerType> imu1Controller_;
    std::shared_ptr<IMU2ControllerType> imu2Controller_;

    std::shared_ptr<IMUPublisherType> imuPublisher_;
    std::shared_ptr<actuators::MobileBaseActuator<MotorControllerType>> mobileBaseActuator_;
    std::shared_ptr<localization::StateEstimationNode> stateEstimationNode_;
};

} // namespace platform::vehicles::waverover
