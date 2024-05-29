#include "platform/sensors/ImuData.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>

namespace platform::sensors {

Eigen::Quaternionf ImuData::getOrientation() const
{
    return orientation;
}

void ImuData::setOrientation(const Eigen::Quaternionf& o)
{
    orientation = o;
}

Eigen::Vector3f ImuData::getAngularVelocity() const
{
    return angular_velocity;
}

void ImuData::setAngularVelocity(const Eigen::Vector3f& av)
{
    angular_velocity = av;
}

Eigen::Vector3f ImuData::getLinearAcceleration() const
{
    return linear_acceleration;
}
void ImuData::setOrientation(const Eigen::Quaterniond& o) {}
void ImuData::setAngularVelocity(const Eigen::Vector3d& av) {}
void ImuData::setLinearAcceleration(const Eigen::Vector3d& la) {}

void ImuData::setLinearAcceleration(const Eigen::Vector3f& la)
{
    linear_acceleration = la;
}

} // namespace platform::sensors
