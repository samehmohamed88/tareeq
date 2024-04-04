#pragma once

#include "platform/devices/DeviceManager.hpp"
#include "platform/io/AsioOperationsImpl.hpp"
#include "platform/io/BoostNetworkIO.hpp"
#include "platform/logging/LoggerFactory.hpp"
#include "platform/vehicle/wave_rover/WaveRoverUtils.hpp"
#include "platform/vehicle/wave_rover/WaveRoverIMUController.hpp"
#include "platform/vehicle/wave_rover/WaveRoverMotorController.hpp"
#include "platform/errors/Errors.hpp"

#include <memory>
#include <utility>

namespace platform::vehicle::waverover {

namespace io = platform::io;
namespace waverover = platform::vehicle::waverover;
namespace motors = platform::motors;
namespace logging = platform::logging;
namespace devices = platform::devices;
namespace imu = platform::sensors::imu;

class WaveRoverManager
{
using BoostNetworkDeviceType = io::BoostNetworkIO<io::AsioOperationsImpl, logging::ConsoleLogger>;
using DeviceManagerType = devices::DeviceManager<BoostNetworkDeviceType, logging::ConsoleLogger>;
using MotorControllerType = motors::MotorController<errors::MotorError, DeviceManagerType, logging::ConsoleLogger>;
using IMUController = sensors::imu::IMUDeviceController<errors::IMUError, DeviceManagerType, logging::ConsoleLogger>;

public:
    WaveRoverManager();

private:

    std::shared_ptr<io::AsioOperationsImpl> asioOperations_;
    std::shared_ptr<logging::ConsoleLogger> logger_;
    std::shared_ptr<BoostNetworkDeviceType> boostNetworkIo_;
    std::shared_ptr<DeviceManagerType> deviceManager_;
    std::unique_ptr<MotorControllerType> motorController_;
    std::unique_ptr<IMUController> imuDeviceController_;

};

} // namespace platform::waverover
