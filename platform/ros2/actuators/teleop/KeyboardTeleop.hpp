#pragma once

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <atomic>
#include <thread>
#include <chrono>

namespace platform::ros2::teleop {

class KeyboardTeleop : public rclcpp::Node {
public:
    KeyboardTeleop()
        : Node("KeyboardTeleop"),
        linear_speed_(0.0),
        angular_speed_(0.0)
    {
        should_exit_ = false;
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);

        setupTerminal();
        setupSignalHandler();

        // Start the input thread
        input_thread_ = std::thread(&KeyboardTeleop::readKeyboardInput, this);
        publish_thread_ = std::thread(&KeyboardTeleop::publishVelocity, this);
    }

    ~KeyboardTeleop() {
        should_exit_ = true;
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
        if (publish_thread_.joinable()) {
            publish_thread_.join();
        }
        tcsetattr(0, TCSANOW, &original_settings_);
    }

    static void signalHandler(int signum) {
        RCLCPP_INFO(rclcpp::get_logger("KeyboardTeleop"), "Signal %d received, exiting...", signum);
        should_exit_ = true;
    }

private:
    void setupTerminal() {
        termios new_settings;
        tcgetattr(0, &original_settings_);
        new_settings = original_settings_;
        new_settings.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(0, TCSANOW, &new_settings);

        int flags = fcntl(0, F_GETFL, 0);
        fcntl(0, F_SETFL, flags | O_NONBLOCK);
    }

    void setupSignalHandler() {
        struct sigaction sigIntHandler;
        sigIntHandler.sa_handler = KeyboardTeleop::signalHandler;
        sigemptyset(&sigIntHandler.sa_mask);
        sigIntHandler.sa_flags = 0;
        sigaction(SIGINT, &sigIntHandler, NULL);
    }

    void readKeyboardInput() {
        char key;
        const double linear_step = 0.1;
        const double angular_step = 0.1;

        while (!should_exit_) {
            if (read(0, &key, 1) > 0) {
                switch (key) {
                    case 0x41: // Up arrow key
                        linear_speed_ += linear_step;
                        break;
                    case 0x42: // Down arrow key
                        linear_speed_ -= linear_step;
                        break;
                    case 0x43: // Right arrow key
                        angular_speed_ -= angular_step;
                        break;
                    case 0x44: // Left arrow key
                        angular_speed_ += angular_step;
                        break;
                }

                linear_speed_ = std::clamp(linear_speed_, -1.0, 1.0);
                angular_speed_ = std::clamp(angular_speed_, -1.0, 1.0);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    void publishVelocity() {
        while (!should_exit_) {
            auto cmd = std::make_unique<geometry_msgs::msg::TwistStamped>();

            cmd->header.stamp = this->get_clock()->now();
            cmd->header.frame_id = "base_link";
            cmd->twist.linear.x = linear_speed_;
            cmd->twist.angular.z = angular_speed_;

            publisher_->publish(std::move(cmd));
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    double linear_speed_;
    double angular_speed_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    std::thread input_thread_;
    std::thread publish_thread_;
    termios original_settings_;
    static std::atomic<bool> should_exit_;
};

//std::atomic<bool> KeyboardTeleop::should_exit_;

} // namespace platform::ros2::teleop
