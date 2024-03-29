#pragma once

#include "platform/sensors/imu/IMUDeviceController.hpp"

#include <memory>
#include <unordered_map>

namespace platform::sensors::imu {

enum class IMUControllerErrors
{
    None,
    NotInitialized,
    SerialPortError,
    InvalidParameter,
    CommunicationError,
};

template<typename DeviceManager, typename ILogger>
class WaveRoverIMUController : public IMUDeviceController<IMUControllerErrors, DeviceManager, ILogger>
{
public:
    WaveRoverIMUController(std::shared_ptr<DeviceManager> deviceManager, std::shared_ptr<const ILogger> logger);

private:
    enum class IMUCommands
    {
        CMD_RETRIEVE_DATA,
    };
    const std::unordered_map<IMUCommands, std::pair<std::string, int>> chassisMovementCommandMap = {
        {IMUCommands::CMD_RETRIEVE_DATA, {"T", 126}}};
};

template<typename DeviceManager, typename ILogger>
WaveRoverIMUController<DeviceManager, ILogger>::WaveRoverIMUController(std::shared_ptr<DeviceManager> deviceManager,
                                                                       std::shared_ptr<const ILogger> logger)
    : IMUDeviceController<IMUControllerErrors, DeviceManager, ILogger>(deviceManager, logger)
{}

} // namespace platform::sensors::imu
