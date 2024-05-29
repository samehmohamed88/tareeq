#pragma once

#include "platform/io/Status.hpp"
#include "platform/sensors/ImuData.hpp"
#include "platform/sensors/zed/ZedCamera.hpp"

#include <sl/Camera.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <iostream>
#include <memory>
#include <optional>

namespace platform::zed {

class ZedImuSensor
{
public:
    ZedImuSensor(std::shared_ptr<ZedCamera> zedCamera)
    : zedCamera_{zedCamera}
    {
        initImu();
    }

    /// @brief Reads data from the ZED Camera 2 SDK.
    /// @return ImuData containing the sensor's current readings.
    std::tuple<platform::io::Status, std::optional<sensors::ImuData>> read();

    bool initImu() {
        // Check if the camera is equipped with an IMU
        if (!zedCamera_->isImuAvailable()) {
            std::cout << "IMU not available on this camera." << std::endl;
            return false;
        }
        return true;
    }

    std::tuple<io::Status, std::optional<sensors::ImuData>> readData()
    {
        auto sensorData = zedCamera_->getImuData();
        if (sensorData.has_value()) {

            auto imuData = zedCamera_->getSensorsData().imu;

            auto pitch_yaw_roll = imuData.pose.getEulerAngles(true); // radians=true

            return {errors::IMUError::NONE,
                    platform::sensors::imu::IMUData{0.0,
                     pitch_yaw_roll[2],
                     pitch_yaw_roll[0],
                     pitch_yaw_roll[1],
                     imuData.linear_acceleration.x,
                     imuData.linear_acceleration.y,
                     imuData.linear_acceleration.z,
                     imuData.angular_velocity.x * (M_PI / 180.0),
                     imuData.angular_velocity.y * (M_PI / 180.0),
                     imuData.angular_velocity.z * (M_PI / 180.0),
                     0.0,
                     0.0,
                     0.0}};
        }
        return {io::Status(io::STATUS::ERROR), std::nullopt};
    };
private:
    std::shared_ptr<ZedCamera> zedCamera_;
};
std::tuple<platform::io::Status, std::optional<platform::sensors::ImuData>> ZedIMUSensor::read()
{
    return {io::Status(io::STATUS::SUCCESS), std::nullopt};
}

} // namespace platform::zed
