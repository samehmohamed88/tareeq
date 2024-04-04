#pragma once

#include "include/nlohmann/json.hpp"
#include "platform/sensors/imu/IMUData.hpp"

#include <string>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

namespace platform::vehicle::waverover::utils {

// Function to convert JSON string to IMUData struct
inline platform::sensors::imu::IMUData jsonToIMUData(const std::string& jsonString)
{
    // Parse the JSON string
    auto json = nlohmann::json::parse(jsonString);

    return {json["temp"].get<double>(),
            json["roll"].get<double>(),
            json["pitch"].get<double>(),
            json["yaw"].get<double>(),
            json["acce_X"].get<double>(),
            json["acce_Y"].get<double>(),
            json["acce_Z"].get<double>(),
            json["gyro_X"].get<double>(),
            json["gyro_Y"].get<double>(),
            json["gyro_Z"].get<double>(),
            json["magn_X"].get<double>(),
            json["magn_Y"].get<double>(),
            json["magn_Z"].get<double>()};
}

enum class WaveRoverCommandID
{
    SPEED_CTRL = 11,
    PWM_INPUT = 1,
    RETRIEVE_IMU_DATA = 71,
    OLED_SCREEN_CONTROL = 3
};

class WaveRoverCommand
{
public:
    using ParamType = std::variant<int, double, std::string>;

    WaveRoverCommand(WaveRoverCommandID waveShareIdentifier,
                     std::unordered_map<std::string, ParamType> parameters,
                     bool writeOnly)
        : waveShareIdentifier_{waveShareIdentifier}
        , parameters_{std::move(parameters)}
        , writeOnly_{writeOnly}
    {}

    const nlohmann::json& toJson() const
    {
        if (isDirty) {
            jsonData = {{"T", waveShareIdentifier_}};
            for (const auto& [key, value] : parameters_) {
                std::visit([this, &key](const auto& val) { this->jsonData[key] = val; }, value);
            }
            jsonString = jsonData.dump(); // Update the string representation as well
            isDirty = false;
        }
        return jsonData;
    }

    const std::string& toJsonString() const
    {
        if (isDirty) {
            toJson(); // This will update jsonString as well
        }
        return jsonString;
    }

    void updateParameter(const std::string& key, const ParamType& value)
    {
        parameters_[key] = value;
        isDirty = true; // Mark as dirty whenever a parameter is updated
    }

    const bool isWriteOnly() const { return writeOnly_; }

private:
    WaveRoverCommandID waveShareIdentifier_;
    std::unordered_map<std::string, ParamType> parameters_;
    bool writeOnly_;
    mutable nlohmann::json jsonData; // Cache for the JSON object
    mutable std::string jsonString;  // Cache for the JSON string representation
    mutable bool isDirty = true;     // Flag to indicate if recalculation is needed
};

class CommandFactory
{
public:
    static WaveRoverCommand createSpeedControlCommand(int leftSpeed, int rightSpeed)
    {
        auto cmd = commands_.find(WaveRoverCommandID::PWM_INPUT)->second;
        return cmd;
    }

    static WaveRoverCommand createRetrieveImuDataCommand()
    {
        auto cmd = commands_.find(WaveRoverCommandID::RETRIEVE_IMU_DATA)->second;
        return cmd;
    }

private:
    static inline std::unordered_map<WaveRoverCommandID, WaveRoverCommand> commands_{
        {WaveRoverCommandID::PWM_INPUT,
         WaveRoverCommand(WaveRoverCommandID::PWM_INPUT,
                          {
                              {"L", 0},
                              {"R", 0},
                          },
                          true)},
        {WaveRoverCommandID::RETRIEVE_IMU_DATA,
         WaveRoverCommand(WaveRoverCommandID::RETRIEVE_IMU_DATA,
                          {

                          },
                          false)},
    };
};

} // namespace platform::vehicle::waverover::utils
