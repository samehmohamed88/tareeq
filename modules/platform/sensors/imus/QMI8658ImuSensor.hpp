#pragma once

#include "platform/io/Status.hpp"
#include "platform/logging/Logger.hpp"
#include "platform/sensors/ImuData.hpp"
#include "platform/sensors/SensorDataFactories.hpp"

#include <concepts>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <deque>

namespace platform::sensors::imu {

/// @brief Represents the QMI8658 IMU sensor, providing initialization, data reading, and validity checks.
class QMI8658ImuSensor
{
public:
    /// @brief Constructs a QMI8658ImuSensor and initializes the sensor.
    QMI8658ImuSensor();

    /// @brief Reads data from the QMI8658 IMU sensor.
    /// @return ImuData containing the sensor's current readings.
//    std::tuple<platform::io::Status, std::optional<ImuData>> read();

    /// @brief Checks whether the sensor is functioning correctly.
    /// @return True if the sensor is valid, False otherwise.
    bool checkValid() { return true; };

//    void processData(const std::string& jsonString);

private:
    /// @brief Initializes the sensor with default settings.
    void initialize()
    {
        // TODO: add some work here
    }

    logging::Logger logger_;
//    std::deque<ImuData> imuDataCache_;
    static constexpr size_t maxCacheSize_ = 5;
};

QMI8658ImuSensor::QMI8658ImuSensor()
    : logger_{"QMI8658ImuSensor"}
{}

//void QMI8658ImuSensor::processData(const std::string& jsonString) {
//    logger_.logDebug("Successfully Retrieved Data from QMI8658 IMU Sensor ");
//    auto imuData = SensorDataFactories::createImuDataFromJsonString(jsonString);
//    imuDataCache_.push_back(imuData);
//}

//std::tuple<platform::io::Status, std::optional<ImuData>> QMI8658ImuSensor::read()
//{
//    std::unique_ptr<ImuData> imuData;
//    logger_.logDebug("Pulling Data from QMI8658 IMU Sensor ");
//    auto [status, jsonString] = deviceDriver_.readImuData();
//    if (status.isSuccess()) {
//        logger_.logDebug("Successfully Retrieved Data from QMI8658 IMU Sensor ");
//        imuData = std::make_unique<ImuData>(SensorDataFactories::createImuDataFromJsonString(jsonString));
//        return {status, imuData};
//    }
//    logger_.logError("Error retrieving Data from QMI8658 Sensor {}", status.toString());
//    return {status, std::nullopt};
//}

} // namespace platform::sensors::imu
