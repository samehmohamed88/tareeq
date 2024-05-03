#include "platform/actuators/SabertoothMotorController.hpp"

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

    return manager_.writeToDevice(port_, command);
}

io::Status SabertoothMotorController::mixedModeDrive(int speed, int turn)
{
    // Speed and turn both range from 0-127
    if (speed < 0 || speed > 127 || turn < 0 || turn > 127) {
        return {io::Status{io::STATUS::ERROR, io::ERROR::INVALID_PARAMETER}};
    }

    int speedCommand = 8;  // Drive forward mixed mode
    int turnCommand = 10;  // Turn right mixed mode
    int addressByte = 128; // default address

    int speedChecksum = calculateChecksum(addressByte, speedCommand, speed);
    int turnChecksum = calculateChecksum(addressByte, turnCommand, turn);

    std::string speedPacket;
    speedPacket += addressByte;
    speedPacket += speedCommand;
    speedPacket += speed;
    speedPacket += speedChecksum;

    std::string turnPacket;
    turnPacket += addressByte;
    turnPacket += turnCommand;
    turnPacket += turn;
    turnPacket += turnChecksum;

    auto status = manager_.writeToDevice(port_, speedPacket);
    if (!status.isSuccess()) {
        return status;
    }

    return manager_.writeToDevice(port_, turnPacket);
}

char SabertoothMotorController::calculateChecksum(int address, int command, int data)
{
    int sum = address + command + data;
    return sum & 127; // Ensure it is a 7-bit checksum
}

} // namespace platform::actuators
