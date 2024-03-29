#pragma once

#include "platform/motors/MotorController.hpp"

namespace platform::motors {

template<typename Error,
         typename DeviceManager,
         typename ILogger>
class WaveRoverMotorController : public MotorController<Error, DeviceManager, ILogger> {
public:
    std::variant<bool, Error> setSpeed(int speed) override;

    std::variant<bool, Error> steer(int angle) override;
};

template<typename Error,
         typename DeviceManager,
         typename ILogger>
std::variant<bool, Error> WaveRoverMotorController<Error, DeviceManager, ILogger>::steer(int angle)
{
    return std::variant<bool, Error>();
}

template<typename Error,
         typename DeviceManager,
         typename ILogger>
std::variant<bool, Error> WaveRoverMotorController<Error, DeviceManager, ILogger>::setSpeed(int speed)
{
    return std::variant<bool, Error>();
}

} // namespace platform::motors
