#pragma once

#include "platform/io/Status.hpp"

#include <sensor_msgs/msg/imu.hpp>
#include <sl/Camera.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <iostream>
#include <optional>
#include <tuple>
#include <cmath>

namespace platform::zed {

class ZedCamera
{
public:
    ZedCamera()
    {
        initParams_.camera_resolution = sl::RESOLUTION::HD720;
        initParams_.camera_fps = 30;
        initParams_.coordinate_units = sl::UNIT::METER;
        initParams_.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;

        // Open the camera
        sl::ERROR_CODE err = zed_.open(initParams_);
        if (err != sl::ERROR_CODE::SUCCESS) {
            std::cout << "Camera Open Error: " << sl::toString(err) << std::endl;
            zed_.close();
        }
    }

    sl::Camera& getCamera() { return zed_; }

    bool isImuAvailable()
    {
        return zed_.getCameraInformation().sensors_configuration.isSensorAvailable(sl::SENSOR_TYPE::ACCELEROMETER);
    }

    std::tuple<io::Status, std::optional<const sensor_msgs::msg::Imu>> getImuData()
    {
        sensor_msgs::msg::Imu imuMsg;
        if (zed_.getSensorsData(sensorsData_, sl::TIME_REFERENCE::CURRENT) == sl::ERROR_CODE::SUCCESS) {

            auto imuData = sensorsData_.imu;

            imuMsg.orientation.x = imuData.pose.getOrientation().ox;
            imuMsg.orientation.y = imuData.pose.getOrientation().oy;
            imuMsg.orientation.z = imuData.pose.getOrientation().oz;
            imuMsg.orientation.w = imuData.pose.getOrientation().ow;

            imuMsg.linear_acceleration.x = imuData.linear_acceleration.x;
            imuMsg.linear_acceleration.y = imuData.linear_acceleration.y;
            imuMsg.linear_acceleration.z = imuData.linear_acceleration.z;

            imuMsg.angular_velocity.x = imuData.angular_velocity.x * (M_PI / 180.0);
            imuMsg.angular_velocity.y = imuData.angular_velocity.y * (M_PI / 180.0);
            imuMsg.angular_velocity.z = imuData.angular_velocity.z * (M_PI / 180.0);

            return {io::Status(io::STATUS::SUCCESS), std::move(imuMsg)};
        }
        return {io::Status(io::STATUS::ERROR), std::nullopt};
    }

private:
    sl::Camera zed_;

    sl::InitParameters initParams_;

    sl::SensorsData sensorsData_;
};

} // namespace platform::zed
