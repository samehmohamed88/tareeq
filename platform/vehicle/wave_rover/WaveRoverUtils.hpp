#pragma once

#include "include/nlohmann/json.hpp"
#include "platform/sensors/imu/IMUData.hpp"

#include <string>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>
#include <cmath>

namespace platform::vehicle::waverover::utils {

/*
 * // TODO: add orientation covariance
imuMsg.linear_acceleration.x = (imuData.acceleration_X / 1000.0) * 9.81;
imuMsg.linear_acceleration.y = (imuData.acceleration_Y / 1000.0) * 9.81;
imuMsg.linear_acceleration.z = (imuData.acceleration_Z / 1000.0) * 9.81;

// TODO: add linear acceleration covariance
imuMsg.angular_velocity.x = imuData.gyro_X * (M_PI / 180.0);
imuMsg.angular_velocity.y = imuData.gyro_Y * (M_PI / 180.0);
imuMsg.angular_velocity.z = imuData.gyro_Z * (M_PI / 180.0);
 */

// Function to convert JSON string to IMUData struct
inline platform::sensors::imu::IMUData jsonToIMUData(const std::string& jsonString)
{
    /// this is empirically found by taking 20 measurements
    // of the IMU stationary on a flat even table
    static constexpr double accelerometerScaleFactor = 1531.80;
    static constexpr double g = 9.81; // Acceleration due to gravity in m/s^2

    // Parse the JSON string
    auto json = nlohmann::json::parse(jsonString);

    return {json["temp"].get<double>(),
            json["roll"].get<double>() * (M_PI / 180.0),
            json["pitch"].get<double>() * (M_PI / 180.0),
            json["yaw"].get<double>() * (M_PI / 180.0),
            (json["acce_X"].get<double>() / accelerometerScaleFactor) * g, // gets acceleration in m/s^2
            (json["acce_Y"].get<double>() / accelerometerScaleFactor) * g, // gets acceleration in m/s^2
            (json["acce_Z"].get<double>() / accelerometerScaleFactor) * g, // gets acceleration in m/s^2
            json["gyro_X"].get<double>() * (M_PI / 180.0),
            json["gyro_Y"].get<double>() * (M_PI / 180.0),
            json["gyro_Z"].get<double>() * (M_PI / 180.0),
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
