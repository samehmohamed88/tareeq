#pragma once

#include "platform/errors/Errors.hpp"
#include "platform/sensors/imu/IMUData.hpp"
#include "platform/sensors/imu/IMUDeviceController.hpp"

#include <sl/Camera.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <iostream>

namespace platform::zed {

template<typename IMUDataType>
class ZedIMUController
{
public:
    ZedIMUController()
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
            //            return 1; // Quit if an error occurred
        }

        // Check if the camera is equipped with an IMU
        if (!zed_.getCameraInformation().sensors_configuration.isSensorAvailable(sl::SENSOR_TYPE::ACCELEROMETER)) {
            std::cout << "IMU not available on this camera." << std::endl;
            zed_.close();
            //            return 1;
        }
    }

    std::tuple<errors::IMUError, std::optional<IMUDataType>> readData()
    {
        if (zed_.getSensorsData(sensorsData_, sl::TIME_REFERENCE::CURRENT) == sl::ERROR_CODE::SUCCESS) {
            // Get the IMU data
            auto imu_data = sensorsData_.imu;

            // Display the accelerometer, gyroscope, and orientation data
            //            std::cout << "Accelerometer: " << imu_data.linear_acceleration << std::endl;
            //            std::cout << "Gyroscope: " << imu_data.angular_velocity << std::endl;
            //            std::cout << "Orientation (Quaternion): " << imu_data.pose.getOrientation() << std::endl;

            auto imuData = sensorsData_.imu;

            auto pitch_yaw_roll = imuData.pose.getEulerAngles(true); // radians=true

            //            return {errors::IMUError::NONE,
            //                    IMUDataType{imuData.linear_acceleration.x,
            //                                imuData.linear_acceleration.y,
            //                                imuData.linear_acceleration.z,
            //                                imuData.angular_velocity.x * (M_PI / 180.0),
            //                                imuData.angular_velocity.y * (M_PI / 180.0),
            //                                imuData.angular_velocity.z * (M_PI / 180.0),
            //                                imuData.pose.getOrientation().ox,
            //                                imuData.pose.getOrientation().oy,
            //                                imuData.pose.getOrientation().oz,
            //                                imuData.pose.getOrientation().ow}};

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
        return {errors::IMUError::FAILURE, std::nullopt};
    };

private:
    //    double getYawFromQuaternion(double x, double y, double z, double w)
    //    {
    //        tf2::Quaternion tf2_quat(x, y, z, w);
    //        tf2::Matrix3x3 m(tf2_quat);
    //        double roll, pitch, yaw;
    //        m.getRPY(roll, pitch, yaw);
    //        return yaw;
    //    }
    // Create a ZED camera object
    sl::Camera zed_;

    // Set initialization parameters
    sl::InitParameters initParams_;

    // Retrieve and return IMU data
    sl::SensorsData sensorsData_;
};

} // namespace platform::zed
