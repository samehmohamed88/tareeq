#pragma once

#include <Eigen/Dense>

#include <utility>

namespace platform::sensors {

/// @class ImuData
/// @brief Encapsulates data from an IMU sensor, including orientation, angular velocity, and linear acceleration.
/// This class uses Eigen library types to store orientation as a quaternion and velocities as 3D vectors, allowing for
/// efficient mathematical operations that are common in sensor data processing and robotics applications.
class ImuData
{
private:
    Eigen::Quaterniond orientation_;     ///< Quaternion representing the orientation.
    Eigen::Vector3d angularVelocity_;    ///< Vector representing angular velocity (rad/s).
    Eigen::Vector3d linearAcceleration_; ///< Vector representing linear acceleration (m/s^2).

public:
    /// Default constructor. Initializes orientation to identity and velocities to zero.
    ImuData()
        : orientation_(Eigen::Quaterniond::Identity())
        , angularVelocity_(Eigen::Vector3d::Zero())
        , linearAcceleration_(Eigen::Vector3d::Zero())
    {}

    ImuData(Eigen::Quaterniond orientation, Eigen::Vector3d angularVelocity, Eigen::Vector3d linearAcceleration)
        : orientation_{std::move(orientation)}
        , angularVelocity_{std::move(angularVelocity)}
        , linearAcceleration_{std::move(linearAcceleration)}
    {}

    /// Constructs ImuData with specified initial values for all components.
    /// @param ox Orientation quaternion x component.
    /// @param oy Orientation quaternion y component.
    /// @param oz Orientation quaternion z component.
    /// @param ow Orientation quaternion w component.
    /// @param avx Angular velocity x component.
    /// @param avy Angular velocity y component.
    /// @param avz Angular velocity z component.
    /// @param lax Linear acceleration x component.
    /// @param lay Linear acceleration y component.
    /// @param laz Linear acceleration z component.
    ImuData(double ox,
            double oy,
            double oz,
            double ow,
            double avx,
            double avy,
            double avz,
            double lax,
            double lay,
            double laz)
        : orientation_(Eigen::Quaterniond(ow, ox, oy, oz))
        , angularVelocity_(Eigen::Vector3d(avx, avy, avz))
        , linearAcceleration_(Eigen::Vector3d(lax, lay, laz))
    {}

    /// Returns the orientation as a quaternion.
    /// @return Eigen::Quaternion of The current orientation.
    Eigen::Quaterniond getOrientation() const;

    /// Sets the orientation to a new quaternion.
    /// @param o New orientation quaternion.
    void setOrientation(const Eigen::Quaterniond& o);

    /// Returns the angular velocity as a 3D vector.
    /// @return Eigen::Vector3d The current angular velocity.
    Eigen::Vector3d getAngularVelocity() const;

    /// Sets the angular velocity to a new 3D vector.
    /// @param av New angular velocity vector.
    void setAngularVelocity(const Eigen::Vector3d& av);

    /// Returns the linear acceleration as a 3D vector.
    /// @return Eigen::Vector3d The current linear acceleration.
    Eigen::Vector3d getLinearAcceleration() const;

    /// Sets the linear acceleration to a new 3D vector.
    /// @param la New linear acceleration vector.
    void setLinearAcceleration(const Eigen::Vector3d& la);
};

} // namespace platform::sensors
