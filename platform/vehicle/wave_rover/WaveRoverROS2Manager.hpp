#pragma once

#include "platform/devices/DeviceManager.hpp"
#include "platform/io/AsioOperationsImpl.hpp"
#include "platform/io/BoostNetworkIO.hpp"
#include "platform/logging/LoggerFactory.hpp"
#include "platform/vehicle/wave_rover/WaveRoverUtils.hpp"
#include "platform/vehicle/wave_rover/WaveRoverIMUController.hpp"
#include "platform/vehicle/wave_rover/WaveRoverMotorController.hpp"
#include "platform/errors/Errors.hpp"
#include "platform/ros2/localization/IMUPublisher.hpp"
#include "platform/ros2/actuators/MobileBaseActuator.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <utility>

namespace platform::vehicle::waverover {

namespace io = platform::io;
namespace vehicles = platform::vehicle;
namespace waverover = platform::vehicle::waverover;
namespace motors = platform::motors;
namespace logging = platform::logging;
namespace devices = platform::devices;
namespace imu = platform::sensors::imu;
namespace actuators = platform::actuators;
namespace localization = platform::ros2::localization;

class WaveRoverROS2Manager
{
using BoostNetworkDeviceType = io::BoostNetworkIO<io::AsioOperationsImpl, logging::ConsoleLogger>;
using DeviceManagerType = devices::DeviceManager<BoostNetworkDeviceType, logging::ConsoleLogger>;
using MotorControllerType = waverover::WaveRoverMotorController<DeviceManagerType, logging::ConsoleLogger>;
using IMUControllerType = waverover::WaveRoverIMUController<DeviceManagerType, logging::ConsoleLogger>;

public:
    WaveRoverROS2Manager();

    void run();

private:
    std::shared_ptr<localization::IMUPublisher<IMUControllerType>> makeIMUPublisher() {
        rclcpp::NodeOptions options;
        // Configure node options with parameters
        options.allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true);

        // Mock or set parameters as required by the node
        options.parameter_overrides({
            {"topic_name", "imu_data"},
            {"imu_frame_id", "imu_link"},
            {"publish_rate_ms", 10000}
        });

        return std::make_shared<localization::IMUPublisher<IMUControllerType>>(imuDeviceController_, options);
    };

    std::shared_ptr<actuators::MobileBaseActuator<MotorControllerType>> makeMobileBaseActuator() {
        rclcpp::NodeOptions options;
        // Configure node options with parameters
        options.allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true);

        // Mock or set parameters as required by the node
        options.parameter_overrides({
            {"topic_name", "cmd_vel"},
            {"queue_size", 10}
        });

        return std::make_shared<actuators::MobileBaseActuator<MotorControllerType>>(motorController_, options);
    };

    vehicles::VehicleConfig vehicleConfig_;
    std::shared_ptr<io::AsioOperationsImpl> asioOperations_;
    std::shared_ptr<logging::ConsoleLogger> logger_;
    std::shared_ptr<BoostNetworkDeviceType> boostNetworkIo_;
    std::shared_ptr<DeviceManagerType> deviceManager_;
    std::shared_ptr<MotorControllerType> motorController_;
    std::shared_ptr<IMUControllerType> imuDeviceController_;

    std::shared_ptr<localization::IMUPublisher<IMUControllerType>> imuPublisher_;
    std::shared_ptr<actuators::MobileBaseActuator<MotorControllerType>> mobileBaseActuator_;

};

} // namespace platform::waverover
