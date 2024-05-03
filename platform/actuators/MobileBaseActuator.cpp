#include "platform/actuators/MobileBaseActuator.hpp"

//void platform::actuators::MobileBaseActuator::velocityCallbackStamped(const geometry_msgs::msg::TwistStamped& msg)
//{
//    processVelocity(msg.twist.linear.x, msg.twist.angular.z);
//}

void platform::actuators::MobileBaseActuator::velocityCallback(const geometry_msgs::msg::Twist& msg)
{
    processVelocity(msg.linear.x, msg.angular.z);
}

void platform::actuators::MobileBaseActuator::processVelocity(double linear, double angular)
{
    RCLCPP_INFO(this->get_logger(),
                "Received Twist: Linear X: '%.2f', Angular Z: '%.2f'",
                linear, angular);

    // Normalize and center speeds to range [0, 127], ensuring 64 is stop for both
    int speed = static_cast<int>((linear / maxLinearSpeed_) * 63.5 + 64);
    int turn = static_cast<int>((angular / maxAngularSpeed_) * 63.5 + 64);

    // Clamping to ensure values are within valid range
    speed = std::clamp(speed, 0, 127);
    turn = std::clamp(turn, 0, 127);

    // Correct command centering:
    // 64 should map directly to 'stop' for both speed and turn
    if (linear == 0 && angular == 0) {
        speed = 64;
        turn = 64;
    }

    // Mixed mode command adjustment (if necessary depending on Sabertooth setup)
    // Here we assume speed command adjusts directly, and turn needs mapping to [-63, +63] from [1, 127]
    sabertoothMotorController_.mixedModeDrive(speed, turn - 64);
}

