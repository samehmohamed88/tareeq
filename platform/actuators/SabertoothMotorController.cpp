#include "platform/actuators/SabertoothMotorController.hpp"
#include "platform/io/Status.hpp"

namespace platform::actuators {
SabertoothMotorController::SabertoothMotorController(const std::string& port, uint32_t baudRate)
    : port_(port)
    , baudRate_(baudRate)
{
    manager_.createDevice(port, baudRate);
}

SabertoothMotorController::~SabertoothMotorController()
{
    manager_.closeDevice(port_);
}

io::Status SabertoothMotorController::driveMotor(int motor, int speed, bool forward)
{
    // Motor IDs: 1 for Motor 1, 2 for Motor 2
    // Speed: 0-127 (0 is stop, 127 is full speed)
    // Forward: true (forward), false (backward)
    if (motor < 1 || motor > 2 || speed < 0 || speed > 127) {
        return {io::Status{io::STATUS::ERROR, io::ERROR::INVALID_PARAMETER}};
    }

    int commandByte = motor == 1 ? (forward ? 0 : 1) : (forward ? 4 : 5);
    int addressByte = 128; // default address
    int checksum = calculateChecksum(addressByte, commandByte, speed);

    std::string command;
    command += addressByte;
    command += commandByte;
    command += speed;
    command += checksum;

    return manager_.writeToDevice(port_, std::move(command));
}

io::Status SabertoothMotorController::sendCommand(int address, int command, int value) {
    char checksum = calculateChecksum(address, command, value);
    std::string packet;
    packet += static_cast<char>(address);
    packet += static_cast<char>(command);
    packet += static_cast<char>(value);
    packet += checksum;
    return manager_.writeToDevice(port_, std::move(packet));
}


io::Status SabertoothMotorController::mixedModeDrive(int speed, int turn) {
    // Ensure the values are within the acceptable range
    if (speed < -127 || speed > 127 || turn < -127 || turn > 127) {
        return {io::Status{io::STATUS::ERROR, io::ERROR::INVALID_PARAMETER}};
    }

    // Determine the correct command and scale the absolute speed value to 0-127
    int forwardCommand = (speed >= 0) ? 8 : 9;
    int speedValue = std::abs(speed);  // No need to scale if already in the range 0-127

    // Determine the correct command for turning and scale the absolute turn value to 0-127
    int turnCommand = (turn >= 0) ? 10 : 11;
    int turnValue = std::abs(turn);  // No need to scale if already in the range 0-127

    // Send speed command
    auto status = sendCommand(128, forwardCommand, speedValue);
    if (!status.isSuccess()) {
        return status;
    }

    // Send turn command
    return sendCommand(128, turnCommand, turnValue);
}


char SabertoothMotorController::calculateChecksum(int address, int command, int data)
{
    int sum = address + command + data;
    return sum & 127; // Ensure it is a 7-bit checksum
}

} // namespace platform::actuators
