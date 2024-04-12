#pragma once

namespace platform::vehicle {
class Chassis {
public:
    Chassis(double rearWheelRadius, double rearWheelSeparation)
        : rearWheelRadius_{rearWheelRadius}
        , rearWheelSeparation_{rearWheelSeparation} {}

    const double getRearWheelRadius() const {
        return rearWheelRadius_;
    }

    const double getRearWheelSeparation() const {
        return rearWheelSeparation_;
    }
private:
    double rearWheelRadius_;
    /// distance between read wheels
    double rearWheelSeparation_;
};
}
