#pragma once

#include "platform/devices/DeviceInterface.hpp"

#include <variant>

namespace platform::motors {

template<typename Error,
         typename DeviceManager,
         typename ILogger>
class MotorController : public devices::DeviceInterface<Error, DeviceManager, ILogger> {
public:

    virtual std::variant<bool, Error> setSpeed(int speed) = 0;

    virtual std::variant<bool, Error> steer(int angle) = 0;
};

} // namespace platform::motors
