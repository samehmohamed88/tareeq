#pragma once

#include <CppLinuxSerial/SerialPort.hpp>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <variant>

namespace av {
namespace motors {

using namespace mn;

class SabertoothDriver
{
public:
    enum class Error
    {
        None,
        NotInitialized,
        SerialPortError,
        InvalidParameter,
        CommunicationError,
    };

    enum class DriveType
    {
        Differential,
        Ackerman,
    };

    explicit SabertoothDriver(const std::string& port, DriveType type);

    std::variant<bool, Error> initialize();

    std::variant<bool, Error> setSpeed(int speed);

    std::variant<bool, Error> steer(int angle);

private:
    CppLinuxSerial::SerialPort serialPort;
    DriveType driveType;
    bool initialized;

    void autobaud();

    void command(int commandByte, int value);

    void motor(int motorNumber, int power);

    void drive(int power);

    void throttleCommand(int commandByte, int power);
};

} // namespace motors
} //  namespace av
