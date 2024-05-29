#pragma once

#include "platform/io/Status.hpp"
#include "platform/logging/LoggerInterface.hpp"
#include "platform/sensors/ImuData.hpp"
#include "platform/sensors/SensorConcepts.hpp"

#include <concepts>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <tuple>
#include <vector>

namespace platform::sensors {

template<typename SensorType, typename SensorData>
class SensorManager
{
private:
    std::vector<SensorType> sensors;

public:
    SensorManager();

    void registerSensor(const SensorType& sensor) {
        sensors.push_back(sensor);
    }

    void deregisterSensor(const SensorType& sensor) {
        sensors.erase(std::remove(sensors.begin(), sensors.end(), sensor), sensors.end());
    }

    std::tuple<platform::io::Status, std::vector<SensorData>> readAll() {
        for (auto& sensor : sensors) {
            auto [status, sensorData] = sensor.read();
        }
    }
};


} // namespace platform::sensors
