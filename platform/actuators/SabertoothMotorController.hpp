#pragma once

#include "platform/io/BoostSerialDeviceManager.hpp"
#include "platform/io/Status.hpp"

#include <boost/asio.hpp>
#include <iostream>
#include <string>

namespace platform::actuators {

/// @brief Class for controlling a Sabertooth motor controller via packetized serial.
class SabertoothMotorController
{
public:
    /// @brief Constructs a Sabertooth motor controller object.
    /// @param port The serial port to connect to, e.g., "/dev/ttyTHS0".
    /// @param baudRate The baud rate for serial communication, defaults to 9600.
    SabertoothMotorController(const std::string& port = "/dev/ttyTHS0", uint32_t baudRate = 9600);

    /// @brief Destroys the Sabertooth motor controller object, closing any open connections.
    ~SabertoothMotorController();

    /// @brief Drives a single motor at a specified speed and direction.
    /// @param motor The motor number to control (1 or 2).
    /// @param speed The speed to set for the motor, ranging from 0 (stop) to 127 (max speed).
    /// @param forward True to drive forward, false for backward.
    io::Status driveMotor(int motor, int speed, bool forward);

    /// @brief Drives motors in mixed mode for combined speed and turning control.
    /// @param speed The speed to drive forward or backward.
    /// @param turn The rate of turn, right or left.
    io::Status mixedModeDrive(int speed, int turn);

private:
    std::string port_; ///< The serial port connected to the Sabertooth controller.
    uint32_t baudRate_; ///< Baud rate for serial communication.
    io::BoostSerialDeviceManager manager_; ///< Manages serial device communication.

    /// @brief Calculates the checksum for packetized serial commands.
    /// @param address The device address byte.
    /// @param command The command byte.
    /// @param data The data byte.
    /// @return The calculated checksum.
    char calculateChecksum(int address, int command, int data);
    io::Status sendCommand(int address, int command, int value);
};

} // namespace platform::actuators
