#pragma once

#include "platform/motors/MotorController.hpp"

#include <unordered_map>

namespace platform::motors {

enum class MotorControllerErrors
{
    None,
    NotInitialized,
    SerialPortError,
    InvalidParameter,
    CommunicationError,
};

template<typename DeviceManager,
         typename ILogger>
class WaveRoverMotorController : public MotorController<MotorControllerErrors, DeviceManager, ILogger>
{
public:
    WaveRoverMotorController(std::shared_ptr<DeviceManager> deviceManager, std::shared_ptr<const ILogger> logger);

    std::variant<bool, MotorControllerErrors> setSpeed(int speed) override;

    std::variant<bool, MotorControllerErrors> steer(int angle) override;

private:
    Command speedControlCommand_;
};

template<typename DeviceManager, typename ILogger>
WaveRoverMotorController<DeviceManager, ILogger>::WaveRoverMotorController(std::shared_ptr<DeviceManager> deviceManager,
                                                                           std::shared_ptr<const ILogger> logger)
    : MotorController<MotorControllerErrors, DeviceManager, ILogger>(deviceManager, logger)
{}

template<typename DeviceManager, typename ILogger>
std::variant<bool, MotorControllerErrors> WaveRoverMotorController<DeviceManager, ILogger>::steer(int angle)
{
    return {};
}

template<typename DeviceManager, typename ILogger>
std::variant<bool, MotorControllerErrors> WaveRoverMotorController<DeviceManager, ILogger>::setSpeed(int speed)
{
    return {};
}

} // namespace platform::motors
