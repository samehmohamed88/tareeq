#pragma once

#include <iostream>
#include <string>
#include <variant>
#include <CppLinuxSerial/SerialPort.hpp>
#include <algorithm>

using namespace mn::CppLinuxSerial;

class SabertoothDriver {
public:
    enum class Error {
        None,
        NotInitialized,
        SerialPortError,
        InvalidParameter,
        CommunicationError,
    };

    enum class DriveType {
        Differential,
        Ackerman,
    };

    explicit SabertoothDriver(const std::string& port, DriveType type)
        : serialPort(port, BaudRate::B_9600, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE),
        driveType(type),
        initialized(false) {}

    std::variant<bool, Error> initialize() {
        try {
            serialPort.Open();
            autobaud();
            initialized = true;
            return true;
        } catch (const SerialPortException& e) {
            std::cerr << "Serial port error: " << e.what() << std::endl;
            return Error::SerialPortError;
        }
    }

    std::variant<bool, Error> setSpeed(int speed) {
        if (!initialized) {
            return Error::NotInitialized;
        }

        // Example usage of speed control, can be adjusted based on drive type
        if (driveType == DriveType::Ackerman) {
            drive(speed);
        } else {
            motor(1, speed);
            motor(2, speed);
        }

        return true;
    }

    std::variant<bool, Error> steer(int angle) {
        if (!initialized) {
            return Error::NotInitialized;
        }

        if (driveType == DriveType::Ackerman) {
            // Ackerman steering logic (not fully implemented here)
        } else {
            // Differential steering logic
            int leftSpeed = angle > 0 ? 100 - angle : 100;
            int rightSpeed = angle < 0 ? 100 + angle : 100;
            motor(1, leftSpeed);
            motor(2, rightSpeed);
        }

        return true;
    }

private:
    SerialPort serialPort;
    DriveType driveType;
    bool initialized;

    void autobaud() const {
        serialPort.Write(std::string(1, 0xAA));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    void command(byte commandByte, byte value) const {
        byte address = 128;
        byte checksum = (address + commandByte + value) & 0x7F;

        std::string command;
        command += static_cast<char>(address);
        command += static_cast<char>(commandByte);
        command += static_cast<char>(value);
        command += static_cast<char>(checksum);

        serialPort.Write(command);
    }

    void motor(byte motorNumber, int power) const {
        if (motorNumber < 1 || motorNumber > 2) return;

        byte commandByte = (motorNumber == 2 ? 4 : 0) + (power < 0 ? 1 : 0);
        throttleCommand(commandByte, power);
    }

    void drive(int power) const {
        throttleCommand(power < 0 ? 9 : 8, power);
    }

    void throttleCommand(byte commandByte, int power) const {
        power = std::clamp(power, -126, 126);
        byte powerByte = static_cast<byte>(std::abs(power));
        command(commandByte, powerByte);
    }
};

int main() {
    SabertoothDriver driver("/dev/serial0", SabertoothDriver::DriveType::Differential);
    auto result = driver.initialize();
    if (std::holds_alternative<SabertoothDriver::Error>(result)) {
        std::cerr << "Initialization failed" << std::endl;
        return -1;
    }

    driver.setSpeed(30); // Set speed
    driver.steer(15);    // Steer with an angle or differential value

    return 0;
}

