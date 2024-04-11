#pragma once

#include "platform/errors/Errors.hpp"
#include "platform/ros2/recording/McapRecorder.hpp"
#include "platform/sensors/imu/IMUDataTypeExtractor.hpp"
#include "platform/sensors/imu/IMUData.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <memory>
#include <random>
#include <string>
#include <thread>

namespace platform::ros2::localization {

using namespace std::chrono_literals;

template<typename IMU1Controller, typename IMU2Controller>
class IMUPublisher : public rclcpp::Node
{
public:
//    using IMU1DataType = typename sensors::imu::IMUDataTypeExtractor<IMU1Controller>::type;
//    using IMU2DataType = typename sensors::imu::IMUDataTypeExtractor<IMU2Controller>::type;
    using IMU1DataType = platform::sensors::imu::IMUData;
    using IMU2DataType = platform::sensors::imu::IMUData;

    IMUPublisher(rclcpp::NodeOptions& options,
                 std::shared_ptr<IMU1Controller> imu1Controller,
                 std::shared_ptr<IMU2Controller> imu2Controller)
        : Node("IMUPublisher", rclcpp::NodeOptions().allow_undeclared_parameters(false))
        , imu1Controller_{imu1Controller}
        , imu2Controller_{imu2Controller}
    {

        // TODO: why are these overrides not working from the caller site?
        this->declare_parameter<std::string>("zed_topic_name", "/zed_imu_data");
        this->declare_parameter<std::string>("rover_topic_name", "/rover_imu_data");
        this->declare_parameter<std::string>("imu_frame_id", "imu_frame_id");
        this->declare_parameter<int>("publish_rate_ms", 250);

        // Now safely access the parameters
        imu1TopicName_ = this->get_parameter("zed_topic_name").as_string();
        imu2TopicName_ = this->get_parameter("rover_topic_name").as_string();
        frameId_ = this->get_parameter("imu_frame_id").as_string();
        publish_rate_ms_ = this->get_parameter("publish_rate_ms").as_int();

        imu1Publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(imu1TopicName_, 10);
        imu2Publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(imu2TopicName_, 10);

        imu1Thread_ = std::thread([this] { this->publishIMU1Loop(); });
        imu2Thread_ = std::thread([this] { this->publishIMU2Loop(); });
    }

private:

    void publishIMU1Loop() {
        rclcpp::WallRate rate(1000ms);
        while (rclcpp::ok()) {
            publishIMU1Data();
            rate.sleep();
        }
    }

    void publishIMU2Loop() {
        rclcpp::WallRate rate(1000ms);
        while (rclcpp::ok()) {
            publishIMU2Data();
            rate.sleep();
        }
    }

    std::shared_ptr<IMU1Controller> imu1Controller_;
    std::shared_ptr<IMU2Controller> imu2Controller_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu1Publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu2Publisher_;

    rclcpp::TimerBase::SharedPtr timer_;
    int publish_rate_ms_ = 0;

    std::string imu1TopicName_;
    std::string imu2TopicName_;
    std::string frameId_;

    // we cache the most recent message because the sensor can drop out quite frequently
    // TODO: find a better alternative
    sensor_msgs::msg::Imu previousImu1Message_;
    sensor_msgs::msg::Imu previousImu2Message_;

    std::thread imu1Thread_;
    std::thread imu2Thread_;

    template<class T>
    void imuDataToRosImuMessageCommon(const T& imuData, sensor_msgs::msg::Imu& imuMsg)
    {
        // TODO: add orientation covariance
        imuMsg.linear_acceleration.x = imuData.acceleration_X;
        imuMsg.linear_acceleration.y = imuData.acceleration_Y;
        imuMsg.linear_acceleration.z = imuData.acceleration_Z;

        // TODO: add linear acceleration covariance
        imuMsg.angular_velocity.x = imuData.gyro_X;
        imuMsg.angular_velocity.y = imuData.gyro_Y;
        imuMsg.angular_velocity.z = imuData.gyro_Z;

        // TODO: add angular velocity covariance
    }

//    void imuDataToRosImuMessage(const IMU1DataType& imuData, sensor_msgs::msg::Imu& imuMsg)
//    {
//        imuMsg.orientation.x = imuData.quaternion.x;
//        imuMsg.orientation.y = imuData.quaternion.y;
//        imuMsg.orientation.z = imuData.quaternion.z;
//        imuMsg.orientation.w = imuData.quaternion.w;
//
//        tf2::Quaternion q;
//
//        imuDataToRosImuMessageCommon(imuData, imuMsg);
//    }

    void imuDataToRosImuMessage(const platform::sensors::imu::IMUData& imuData, sensor_msgs::msg::Imu& imuMsg)
    {
        // imu data from with Euler angles need to be converted
        // to quaterion
        tf2::Quaternion q;
        q.setRPY(imuData.roll, imuData.pitch, imuData.yaw);
        imuMsg.orientation = tf2::toMsg(q);

        imuDataToRosImuMessageCommon(imuData, imuMsg);
    }

    template<typename Controller>
    void publishIMUData(const std::string& topicName,
                        sensor_msgs::msg::Imu& previousImuMessage,
                        Controller& imuController,
                        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher)
    {
        try {
            sensor_msgs::msg::Imu imuMsg;
            imuMsg.header.frame_id = frameId_;

            // Populate IMU data here using imuController
            auto [error, imuData] = imuController->readData();

            if (error == errors::IMUError::NONE) {
                if (imuData.has_value()) {
                    imuDataToRosImuMessage(*imuData, imuMsg);
                    imuMsg.header.stamp = this->get_clock()->now();
                    previousImuMessage = imuMsg;
                }
            } else {
                imuMsg = previousImuMessage;
                imuMsg.header.stamp = this->get_clock()->now();
            }

            imuPublisher->publish(imuMsg);
        } catch (const std::exception& e) {
            // Handle standard exceptions
            std::cerr << "Standard exception: " << e.what() << std::endl;
            std::cout << " at the very least publish previous message no matter what " << std::endl;
            imuPublisher->publish(previousImuMessage);
        } catch (...) {
            // Handle any other exceptions
            std::cerr << "An unknown exception occurred." << std::endl;
            std::cout << " at the very least publish previous message no matter what " << std::endl;
            imuPublisher->publish(previousImuMessage);
        }
    }

    void publishIMU1Data()
    {
        publishIMUData(imu1TopicName_, previousImu1Message_, imu1Controller_, imu1Publisher_);
    }

    void publishIMU2Data()
    {
        std::cout << "Publishing Wave Rover IMU Data" << std::endl;
        publishIMUData(imu2TopicName_, previousImu2Message_, imu2Controller_, imu2Publisher_);
    }
};

} // namespace platform::ros2::localization
