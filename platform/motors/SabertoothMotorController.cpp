//
//#include "av/motors/SabertoothMotorController.hpp"
//
//#include <algorithm>
//#include <chrono>
//#include <iostream>
//#include <string>
//#include <thread>
//#include <variant>
//
//namespace av {
//namespace motors {
//
//SabertoothDriver::SabertoothDriver(const std::string& port, DriveType type)
//    : serialPort(port,
//                 CppLinuxSerial::BaudRate::B_9600,
//                 CppLinuxSerial::NumDataBits::EIGHT,
//                 CppLinuxSerial::Parity::NONE,
//                 CppLinuxSerial::NumStopBits::ONE)
//    , driveType(type)
//    , initialized(false)
//{}
//
//std::variant<bool, SabertoothDriver::Error> SabertoothDriver::initialize()
//{
//    try {
//        serialPort.Open();
//        autobaud();
//        initialized = true;
//        return true;
//    } catch (const CppLinuxSerial::Exception& e) {
//        std::cerr << "Serial port error: " << e.what() << std::endl;
//        return Error::SerialPortError;
//    }
//}
//
//std::variant<bool, SabertoothDriver::Error> SabertoothDriver::setSpeed(int speed)
//{
//    if (!initialized) {
//        return Error::NotInitialized;
//    }
//
//    // Example usage of speed control, can be adjusted based on drive type
//    if (driveType == DriveType::Ackerman) {
//        drive(speed);
//    } else {
//        motor(1, speed);
//        motor(2, speed);
//    }
//
//    return true;
//}
//
//std::variant<bool, SabertoothDriver::Error> SabertoothDriver::steer(int angle)
//{
//    if (!initialized) {
//        return Error::NotInitialized;
//    }
//
//    if (driveType == DriveType::Ackerman) {
//        // Ackerman steering logic (not fully implemented here)
//    } else {
//        // Differential steering logic
//        int leftSpeed = angle > 0 ? 100 - angle : 100;
//        int rightSpeed = angle < 0 ? 100 + angle : 100;
//        motor(1, leftSpeed);
//        motor(2, rightSpeed);
//    }
//
//    return true;
//}
//
//void SabertoothDriver::autobaud()
//{
//    const std::string cmd(std::string(1, 0xAA));
//    serialPort.Write(cmd);
//    std::this_thread::sleep_for(std::chrono::milliseconds(500));
//}
//
//void SabertoothDriver::command(int commandByte, int value)
//{
//    int address = 128;
//    int checksum = (address + commandByte + value) & 0x7F;
//
//    std::string command;
//    command += static_cast<char>(address);
//    command += static_cast<char>(commandByte);
//    command += static_cast<char>(value);
//    command += static_cast<char>(checksum);
//
//    serialPort.Write(command);
//}
//
//void SabertoothDriver::motor(int motorNumber, int power)
//{
//    if (motorNumber < 1 || motorNumber > 2)
//        return;
//
//    int commandByte = (motorNumber == 2 ? 4 : 0) + (power < 0 ? 1 : 0);
//    throttleCommand(commandByte, power);
//}
//
//void SabertoothDriver::drive(int power)
//{
//    throttleCommand(power < 0 ? 9 : 8, power);
//}
//
//void SabertoothDriver::throttleCommand(int commandByte, int power)
//{
//    power = std::clamp(power, -126, 126);
//    //        int powerByte = static_cast<byte>(std::abs(power));
//    command(commandByte, power);
//}
//
//} // namespace motors
//} //  namespace av
