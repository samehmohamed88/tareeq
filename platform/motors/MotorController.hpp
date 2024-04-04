#pragma once

#include "platform/vehicle/config/VehicleConfig.hpp"
#include "platform/devices/DeviceInterface.hpp"

#include <variant>

namespace platform::motors {

template<typename Error, typename DeviceManager, typename ILogger>
class MotorController : public devices::DeviceInterface<Error, ILogger>
{
public:
    MotorController(std::shared_ptr<DeviceManager> deviceManager, std::shared_ptr<const ILogger> logger, const vehicle::VehicleConfig& vehicleConfig);

    virtual std::variant<bool, Error> initialize() = 0;

    virtual std::variant<bool, Error> stop() = 0;

    virtual std::variant<bool, Error> setVelocity(double linearVelocity, double angularVelocity) = 0;

protected:
    virtual std::variant<bool, Error> setMotorPwm(int leftMotorPwm, int rightMotorPwm) = 0;

    std::shared_ptr<DeviceManager> deviceManager_;
    const vehicle::VehicleConfig& vehicleConfig_;

};

template<typename Error, typename DeviceManager, typename ILogger>
MotorController<Error, DeviceManager, ILogger>::MotorController(std::shared_ptr<DeviceManager> deviceManager,
                                                                std::shared_ptr<const ILogger> logger, const vehicle::VehicleConfig& vehicleConfig)
    : devices::DeviceInterface<Error, ILogger>(logger)
    , deviceManager_{deviceManager}
    , vehicleConfig_{vehicleConfig} {}

} // namespace platform::motors
