#pragma once

#include "platform/sensors/imu/IMUData.hpp"
#include "platform/errors/Errors.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <memory>
#include <random>
#include <string>

namespace platform::ros2::localization {

using namespace std::chrono_literals;


template <typename IMUController>
class IMUPublisher : public rclcpp::Node
{
public:
    IMUPublisher(std::shared_ptr<IMUController> imuController, rclcpp::NodeOptions& options)
        : Node("IMUPublisher", rclcpp::NodeOptions().allow_undeclared_parameters(false))
        , imuController_{imuController}
    {

        // Declare parameters with default values as needed
        this->declare_parameter<std::string>("topic_name", "default_imu_topic");
        this->declare_parameter<std::string>("imu_frame_id", "default_imu_frame_id");
        this->declare_parameter<int>("publish_rate_ms", 1000);

        // Now safely access the parameters
        topic_name_ = this->get_parameter("topic_name").as_string();
        frame_id_ = this->get_parameter("imu_frame_id").as_string();
        publish_rate_ms_ = this->get_parameter("publish_rate_ms").as_int();

        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(topic_name_, 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(publish_rate_ms_),
            [this] { this->publishIMUData(); }
        );
    }

private:

    std::shared_ptr<IMUController> imuController_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int publish_rate_ms_ = 0;
    std::string topic_name_;
    std::string frame_id_;

    void imuDataToRosImuMessage(const sensors::imu::IMUData& imu_data, sensor_msgs::msg::Imu& imu_msg) {
        // 1. Convert Euler angles (roll, pitch, yaw) to a quaternion
        tf2::Quaternion q;
        q.setRPY(imu_data.roll, imu_data.pitch, imu_data.yaw);

        // 2. Assign the quaternion to the ROS message
        imu_msg.orientation = tf2::toMsg(q);

        // TODO: add orientation covariance

        // 3. Assign the linear acceleration and angular velocity
        imu_msg.linear_acceleration.x = imu_data.acceleration_X;
        imu_msg.linear_acceleration.y = imu_data.acceleration_Y;
        imu_msg.linear_acceleration.z = imu_data.acceleration_Z;

        // TODO: add linear acceleration covariance

        imu_msg.angular_velocity.x = imu_data.gyro_X;
        imu_msg.angular_velocity.y = imu_data.gyro_Y;
        imu_msg.angular_velocity.z = imu_data.gyro_Z;

        // TODO: add angular velocity covariance
    }

    void publishIMUData()
    {
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = frame_id_;

        // Populate IMU data here using imuController_
        auto [error, data] = imuController_->readData();

        if (error == errors::IMUError::NONE) {
            if (data.has_value()) {
                imuDataToRosImuMessage(*data, imu_msg);
            }
        }
        publisher_->publish(imu_msg);
//        RCLCPP_INFO(this->get_logger(), "Publishing IMU data: #%d", count_++);
    }
};

} // namespace platform::ros2::localization
