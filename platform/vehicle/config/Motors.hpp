#pragma once

namespace platform::vehicle {
class Motor
{
public:
    Motor(double pwmThreshold)
        : pwmThreshold_{pwmThreshold}
    {}

    const double getPwmThreshold() const {
        return pwmThreshold_;
    }

private:
    double pwmThreshold_;
};
} // namespace platform::vehicle
