#pragma once

#include "platform/vehicle/config/Chassis.hpp"
#include "platform/vehicle/config/Motors.hpp"

namespace platform::vehicle {

class VehicleConfig {
public:

    VehicleConfig(double rearWheelRadius, double rearWheelSeparation, double pwmThreshold)
        : chassis_{rearWheelRadius, rearWheelSeparation}
        , motor_{pwmThreshold} {}

    const Chassis& getChassisConfig() const {
        return chassis_;
    }

    const Motor& getMotorConfig() const {
        return motor_;
    }
private:
    Chassis chassis_;
    Motor motor_;
};

} // namespace platform::vehicles
