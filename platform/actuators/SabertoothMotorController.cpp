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

io::Status SabertoothMotorController::mixedModeDrive(int speed, int turn) {
    if (speed < 0 || speed > 127 || turn < 0 || turn > 127) {
        return {io::Status{io::STATUS::ERROR, io::ERROR::INVALID_PARAMETER}};
    }

    // Translate the incoming speed and turn values to the appropriate command data.
    int driveCommand = (speed == 0) ? 64 : speed;
    int turnCommandData = (turn == 0) ? 64 : turn;

    int speedCommand = 8;  // Drive forward mixed mode
    int turnCommand = 10;  // Turn right mixed mode
    int addressByte = 128; // Default address from the DIP switches

    int speedChecksum = calculateChecksum(addressByte, speedCommand, driveCommand);
    int turnChecksum = calculateChecksum(addressByte, turnCommand, turnCommandData);

    std::string speedPacket;
    speedPacket += static_cast<char>(addressByte);
    speedPacket += static_cast<char>(speedCommand);
    speedPacket += static_cast<char>(driveCommand);
    speedPacket += static_cast<char>(speedChecksum);

    std::string turnPacket;
    turnPacket += static_cast<char>(addressByte);
    turnPacket += static_cast<char>(turnCommand);
    turnPacket += static_cast<char>(turnCommandData);
    turnPacket += static_cast<char>(turnChecksum);

    auto status = manager_.writeToDevice(port_, std::move(speedPacket));
    if (!status.isSuccess()) {
        return status;
    }

    return manager_.writeToDevice(port_, std::move(turnPacket));
}


char SabertoothMotorController::calculateChecksum(int address, int command, int data)
{
    int sum = address + command + data;
    return sum & 127; // Ensure it is a 7-bit checksum
}

} // namespace platform::actuators
