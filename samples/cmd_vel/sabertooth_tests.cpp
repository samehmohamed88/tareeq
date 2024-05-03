#include "platform/actuators/SabertoothMotorController.hpp"
#include <iostream>
#include <thread>  // For std::this_thread::sleep_for
#include <chrono>  // For std::chrono::seconds

// Function to drive motors directly
void driveMotorsDirectly(platform::actuators::SabertoothMotorController& sabertooth) {
    std::cout << "Driving motors forward directly..." << std::endl;
    sabertooth.driveMotor(1, 33, true);  // Motor 1 forward
    sabertooth.driveMotor(2, 33, true);  // Motor 2 forward
    std::this_thread::sleep_for(std::chrono::seconds(5));  // Run motors for 5 seconds

    std::cout << "Driving motors backward directly..." << std::endl;
    sabertooth.driveMotor(1, 33, false);  // Motor 1 backward
    sabertooth.driveMotor(2, 33, false);  // Motor 2 backward
    std::this_thread::sleep_for(std::chrono::seconds(5));  // Run motors for 5 seconds

    // Optionally, stop the motors
    std::cout << "Stopping motors..." << std::endl;
    sabertooth.driveMotor(1, 0, true);  // Stop Motor 1
    sabertooth.driveMotor(2, 0, true);  // Stop Motor 2
}

// Function to drive motors in mixed mode
void driveMotorsMixedMode(platform::actuators::SabertoothMotorController& sabertooth) {
    std::cout << "Driving forward in mixed mode..." << std::endl;
    sabertooth.mixedModeDrive(33, 0);  // Drive forward at quarter speed, no turn
    std::this_thread::sleep_for(std::chrono::seconds(5));

    std::cout << "Driving backward in mixed mode..." << std::endl;
    sabertooth.mixedModeDrive(33, 0);  // Drive backward at quarter speed, no turn
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // Optionally, stop the motors
    std::cout << "Stopping motors in mixed mode..." << std::endl;
    sabertooth.mixedModeDrive(0, 0);  // Stop driving and turning
}

int main() {
    try {
        // Initialize the Sabertooth motor controller on the specified port and baud rate
        platform::actuators::SabertoothMotorController sabertooth("/dev/ttyTHS0", 9600);

        // Test direct motor control
        driveMotorsDirectly(sabertooth);

        // Test mixed mode drive
        driveMotorsMixedMode(sabertooth);

    } catch (const std::exception& e) {
        std::cerr << "An exception occurred: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
