#pragma once

#include "platform/devices/DeviceInterface.hpp"

#include <variant>

namespace platform::motors {

template<typename Error, typename DeviceManager, typename ILogger>
class MotorController : public devices::DeviceInterface<Error, ILogger>
{
public:
    MotorController(std::shared_ptr<DeviceManager> deviceManager, std::shared_ptr<const ILogger> logger);

    virtual std::variant<bool, Error> initialize() = 0;

    virtual std::variant<bool, Error> stop() = 0;

    virtual std::variant<bool, Error> setWheelSpeeds(double leftWheelSpeed, double rightWheelSpeed) = 0;

protected:
    std::shared_ptr<DeviceManager> deviceManager_;
};

template<typename Error, typename DeviceManager, typename ILogger>
MotorController<Error, DeviceManager, ILogger>::MotorController(std::shared_ptr<DeviceManager> deviceManager,
                                                                std::shared_ptr<const ILogger> logger)
    : devices::DeviceInterface<Error, ILogger>(logger)
    , deviceManager_{deviceManager}
{}

} // namespace platform::motors
