#pragma once

#include <nlohmann/json.hpp>

#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

namespace platform::waverover::utils {

class CommandType {
public:
    class CMD_SPEED_CTRL {
    public:
        static const int ID = 1;
        static inline const std::string LeftMotor = "L";
        static inline const std::string RightMotor = "R";
    };

    class CMD_PWM_INPUT {
    public:
        static const int ID = 11;
        static inline const std::string LeftMotor = "L";
        static inline const std::string RightMotor = "R";
    };

    class RETRIEVE_IMU_DATA {
    public:
        static const int ID = 126;
    };

    class OLED_SCREEN_CONTROL {
    public:
        static const int ID = 3;
    };
};

struct Command {

    using ParamType = std::variant<int, double, std::string>;

    int waveShareIdentifier{};

    std::unordered_map<std::string, ParamType> parameters;
};

class CommandFactory {
public:
    static Command createSpeedControlCommand(double leftSpeed, double rightSpeed) {
        Command cmd;
        cmd.waveShareIdentifier = CommandType::CMD_SPEED_CTRL::ID;
        cmd.parameters[CommandType::CMD_SPEED_CTRL::LeftMotor] = leftSpeed;
        cmd.parameters[CommandType::CMD_SPEED_CTRL::RightMotor] = rightSpeed;
        return cmd;
    }

    static Command createPwmInputCommand(double leftValue, double rightValue) {
        Command cmd;
        cmd.waveShareIdentifier = CommandType::CMD_PWM_INPUT::ID;
        cmd.parameters[CommandType::CMD_PWM_INPUT::LeftMotor] = leftValue;
        cmd.parameters[CommandType::CMD_PWM_INPUT::RightMotor] = rightValue;
        return cmd;
    }

    static Command createRetrieveImuDataCommand() {
        Command cmd;
        cmd.waveShareIdentifier = CommandType::RETRIEVE_IMU_DATA::ID;
        // No additional parameters for this command
        return cmd;
    }

private:

};


}
