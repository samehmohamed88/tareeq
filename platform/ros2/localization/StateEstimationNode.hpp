#pragma once

#include "av/localization/EKF.hpp"

#include <Eigen/Dense>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <iostream>
#include <memory>
#include <numeric>
#include <queue>
#include <vector>

namespace platform::ros2::localization {

#include <algorithm>
#include <deque>
#include <numeric>

class VelocityEstimator
{
public:
    explicit VelocityEstimator(size_t window_size = 3)
        : window_size_(window_size)
    {}

    void update(double acceleration, double gyro_z, double delta_t)
    {
        // Update acceleration buffer and compute average
        if (acceleration_buffer_.size() >= window_size_) {
            acceleration_buffer_.pop_front();
        }
        acceleration_buffer_.push_back(acceleration);
        double average_acceleration = std::accumulate(acceleration_buffer_.begin(), acceleration_buffer_.end(), 0.0) /
                                      acceleration_buffer_.size();

        // Update gyro buffer and compute average
        if (gyro_z_buffer_.size() >= window_size_) {
            gyro_z_buffer_.pop_front();
        }
        gyro_z_buffer_.push_back(gyro_z);
        average_gyro_z_ = std::accumulate(gyro_z_buffer_.begin(), gyro_z_buffer_.end(), 0.0) /
                          gyro_z_buffer_.size();

        // Integrate acceleration to estimate velocity
        velocity_ += average_acceleration * delta_t;
    }

    double get_velocity() const { return velocity_; }
    double get_average_gyro_z() const { return average_gyro_z_; }

private:
    std::deque<double> acceleration_buffer_; // Stores the recent acceleration values for averaging
    std::deque<double> gyro_z_buffer_;       // Stores the recent gyro Z values for averaging
    size_t window_size_;                     // Number of samples to average for both acceleration and gyro
    double velocity_{0.0};                   // Estimated velocity
    double average_gyro_z_{0.0};             // Average gyro Z value
};

class StateEstimationNode : public rclcpp::Node
{
public:
    StateEstimationNode()
        : Node("StateEstimationNode")
    {

        imu1Subscription_.subscribe(this, "/zed_imu_data");
        imu2Subscription_.subscribe(this, "/rover_imu_data");
        velocitySubscription_.subscribe(this, "/cmd_vel");

        twoIMUControllersSync_.reset(new message_filters::Synchronizer<TwoIMUApproximateTimePolicy>(
            TwoIMUApproximateTimePolicy(10), imu1Subscription_, imu2Subscription_, velocitySubscription_));
        twoIMUControllersSync_->registerCallback(
            std::bind(&StateEstimationNode::twoIMUCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom/filtered", 10);

        last_update_ = std::chrono::steady_clock::now();
    };

private:
    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& quat)
    {
        tf2::Quaternion tf2_quat(quat.x, quat.y, quat.z, quat.w);
        tf2::Matrix3x3 m(tf2_quat);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

    void twoIMUCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& imu1Message,
                  const sensor_msgs::msg::Imu::ConstSharedPtr& imu2Message,
                  const geometry_msgs::msg::TwistStamped::ConstSharedPtr& velocityMessage)
    {
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = now - last_update_;
        double delta_t = elapsed_seconds.count();

        last_update_ = now; // Update the last_update_ time


        double linearVelocityControlCmd = velocityMessage->twist.linear.x;
        double angularVelocityControlCmd = velocityMessage->twist.angular.z;

        // run EK Predict
        ekf_.predict(linearVelocityControlCmd, angularVelocityControlCmd, delta_t);

        imu1LinearVelocityEstimator_.update(imu1Message->linear_acceleration.x, delta_t, imu1Message->angular_velocity.z);
        double imu1LinearAccelerationAverageMeasured = imu1LinearVelocityEstimator_.get_velocity();
        double imu1AngularVelocityMeasured = imu1LinearVelocityEstimator_.get_average_gyro_z();

        imu2LinearVelocityEstimator_.update(imu2Message->linear_acceleration.x, delta_t, imu2Message->angular_velocity.z);
        double imu2LinearAccelerationAverageMeasured = imu2LinearVelocityEstimator_.get_velocity();
        double imu2AngularVelocityMeasured = imu2LinearVelocityEstimator_.get_average_gyro_z();

        Eigen::MatrixXd Z(6, 1); // Assuming you have 4 measurements as per your H matrix

        Z(0, 0) = getYawFromQuaternion(imu1Message->orientation);
        Z(1, 0) = imu1LinearAccelerationAverageMeasured;
        Z(2, 0) = imu1AngularVelocityMeasured;

        Z(3, 0) = getYawFromQuaternion(imu2Message->orientation);
        Z(4, 0) = imu2LinearAccelerationAverageMeasured;
        Z(5, 0) = imu2AngularVelocityMeasured;

        ekf_.correct(Z);

        // Process synchronized messages
        std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << std::endl;
        std::cout << "UPDATING ODOMETRY" << std::endl;

        auto currentState = ekf_.getState();
        auto covariance = ekf_.getCovariance();

        double theta_ = currentState(2, 0);

        nav_msgs::msg::Odometry odom;
        odom.header.frame_id = "map";
        odom.header.stamp = this->get_clock()->now();
        odom.child_frame_id = "base_link";

        odom.pose.pose.orientation.x = 0;
        odom.pose.pose.orientation.y = 0;
        odom.pose.pose.orientation.z = std::sin(theta_ / 2.0);
        odom.pose.pose.orientation.w = std::sin(theta_ / 2.0);
        odom.pose.pose.position.x = currentState(0, 0);
        odom.pose.pose.position.y = currentState(1, 0);
        odom.pose.pose.position.z = 0;

        // Map the top-left 3x3 block of the 5x5 covariance matrix
        // to the top-left 3x3 block of the 6x6 Odometry pose covariance matrix
        // pose.covariance array is a 6x6 matrix stored in row-major order
        for (size_t i = 0; i < 3; i++) {
            for (size_t j = 0; j < 3; j++) {
                odom.pose.covariance[i * 6 + j] = covariance(i, j);
            }
        }

        odom.twist.twist.linear.x = currentState(3, 0);
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.linear.z = 0;
        odom.twist.twist.angular.x = 0;
        odom.twist.twist.angular.y = 0;
        odom.twist.twist.angular.z = currentState(4, 0);

        odom.twist.covariance[0] = covariance(3, 3);
        // twist.covariance array is a 6x6 matrix stored in row-major order
        size_t i = 5, j = 5;
        odom.twist.covariance[i * 6 + j] = covariance(4, 4);

        publisher_->publish(odom);

        std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << std::endl;
        std::cout << "PUBLISHED ODOMETRY FILTERED" << std::endl;
    }

//    typedef message_filters::sync_policies::
//        ApproximateTime<sensor_msgs::msg::Imu, geometry_msgs::msg::TwistStamped>
//            OneIMUApproximateTimePolicy;

    typedef message_filters::sync_policies::
        ApproximateTime<sensor_msgs::msg::Imu, sensor_msgs::msg::Imu, geometry_msgs::msg::TwistStamped>
            TwoIMUApproximateTimePolicy;

    message_filters::Subscriber<sensor_msgs::msg::Imu> imu1Subscription_;

    message_filters::Subscriber<sensor_msgs::msg::Imu> imu2Subscription_;

    message_filters::Subscriber<geometry_msgs::msg::TwistStamped> velocitySubscription_;

//    std::unique_ptr<message_filters::Synchronizer<OneIMUApproximateTimePolicy>> oneIMUControllerSync_;
    std::unique_ptr<message_filters::Synchronizer<TwoIMUApproximateTimePolicy>> twoIMUControllersSync_;

    av::localization::EKF ekf_;
    VelocityEstimator imu1LinearVelocityEstimator_;
    VelocityEstimator imu2LinearVelocityEstimator_;
    std::chrono::time_point<std::chrono::steady_clock> last_update_; // Class variable to store the time point
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
};

} // namespace platform::ros2::localization
