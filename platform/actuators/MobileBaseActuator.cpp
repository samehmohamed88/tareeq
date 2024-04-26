#include "platform/actuators/MobileBaseActuator.hpp"

void platform::actuators::MobileBaseActuator::velocityCallback(const geometry_msgs::msg::TwistStamped& msg)
{

    RCLCPP_INFO(this->get_logger(),
                "Received Twist: Linear X: '%.2f', Angular Z: '%.2f'",
                msg.twist.linear.x,
                msg.twist.angular.z);

    auto result = hardwareInterface_(msg.twist.linear.x, msg.twist.angular.z);
    if (result.isError()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Error occurred while setting velocity : %s",
                     result.toString().c_str());
    }
}
