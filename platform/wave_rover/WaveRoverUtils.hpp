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

private:
    mutable nlohmann::json jsonData; // Cache for the JSON object
    mutable std::string jsonString;  // Cache for the JSON string representation
    mutable bool isDirty = true;     // Flag to indicate if recalculation is needed

public:
    const nlohmann::json& toJson() const {
        if (isDirty) {
            jsonData = {{"T", waveShareIdentifier}};
            for (const auto& [key, value] : parameters) {
                jsonData[key] = value;
            }
            jsonString = jsonData.dump(4);  // Update the string representation as well
            isDirty = false;
        }
        return jsonData;
    }

    const std::string& toJsonString() const {
        if (isDirty) {
            toJson();  // This will update jsonString as well
        }
        return jsonString;
    }

    void updateParameter(const std::string& key, const ParamType& value) {
        parameters[key] = value;
        isDirty = true;  // Mark as dirty whenever a parameter is updated
    }
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
