#include "platform/actuators/MobileBaseActuator.hpp"

void platform::actuators::MobileBaseActuator::velocityCallback(const geometry_msgs::msg::TwistStamped& msg)
{

    RCLCPP_INFO(this->get_logger(),
                "Received Twist: Linear X: '%.2f', Angular Z: '%.2f'",
                msg.twist.linear.x,
                msg.twist.angular.z);

    double linearVelocity = normalizeToHalfRange(msg.twist.linear.x);
    double angularVelocity = normalizeToHalfRange(msg.twist.angular.z);

    RCLCPP_INFO(this->get_logger(),
                "Normalized Velocities.  Linear: '%.2f', Angular Z: '%.2f'",
                linearVelocity,
                angularVelocity);

    double leftWheelSpeed = ((linearVelocity - angularVelocity) * (rearWheelSeparation_ / 2)) / rearWheelRadius_;
    double rightWheelSpeed = ((linearVelocity + angularVelocity) * (rearWheelSeparation_ / 2)) / rearWheelRadius_;

    RCLCPP_INFO(this->get_logger(),
                "Wheel Speeds.  Left: '%.2f', Right: '%.2f'",
                leftWheelSpeed,
                rightWheelSpeed);

    leftWheelSpeed = std::clamp(leftWheelSpeed, -maxLinearSpeed_, maxLinearSpeed_);
    rightWheelSpeed = std::clamp(rightWheelSpeed, -maxLinearSpeed_, maxLinearSpeed_);

    auto request = std::string(R"({"T":1,"L":)" + formatDouble(leftWheelSpeed) +
                               ",\"R\":" + formatDouble(rightWheelSpeed) + "}\n");

    RCLCPP_INFO(this->get_logger(), "Created Request String: '%s'", request.c_str());

    boostDeviceManager_.writeToDevice("/dev/ttyUSB0", request);

    //    hardwareInterface_(msg.twist.linear.x, msg.twist.angular.z);
    //    auto result = hardwareInterface_(msg.twist.linear.x, msg.twist.angular.z);
    //    if (result.isError()) {
    //        RCLCPP_ERROR(this->get_logger(),
    //                     "Error occurred while setting velocity : %s",
    //                     result.toString().c_str());
    //    }
}
