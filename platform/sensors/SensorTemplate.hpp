#pragma once

#include "platform/sensors/SensorConcepts.hpp"

namespace platform::sensors {

template<ReadableSensor SensorType>
class SensorTemplate {
public:
    void readSensorData() {
        SensorType sensor;
        sensor.read();
        if constexpr (CalibratableSensor<SensorType>) {
            sensor.calibrate();  // Conditionally compile this path
        }
    }
};

} // namespace platform::sensors
