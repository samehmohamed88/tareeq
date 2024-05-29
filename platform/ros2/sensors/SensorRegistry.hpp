#pragma once

#include "platform/sensors/Sensor.hpp"
#include "platform/sensors/imus/QMI8658ImuSensor.hpp"
// #include "platform/sensors/zed/ZedVisualOdometrySensor.hpp"
// #include "platform/sensors/zed/ZedIMUSensor.hpp"
#include "platform/vehicles/wave_rover/WaveRoverImuReader.hpp"

#include <algorithm>
#include <iostream>
#include <utility>
#include <variant>
#include <vector>

namespace platform::sensors {

class SensorRegistry
{
public:
    SensorRegistry(vehicles::waverover::WaveRoverImuReader waveRoverImuReader)
    {
        auto sensor = imu::QMI8658ImuSensor();
        registerSensor(sensor, std::move(waveRoverImuReader), [](){
            std::cout << "PUBLISHED" << std::endl;
        });
    }

    template<typename SensorT, typename DataReader, typename DataPublisher>
    void registerSensor(SensorT sensor, DataReader reader, DataPublisher publisher);

    void getImuData()
    {
        std::cout << "In Sensor Registry" << std::endl;
        auto sensor = sensors_[0];
        std::cout << "In Sensor Registry : Calling Read Sensor" << std::endl;
        publishSensorData(sensor);
    };

private:
    std::vector<Sensor> sensors_;
};

template<typename SensorT, typename DataReader, typename DataPublisher>
void SensorRegistry::registerSensor(SensorT sensor, DataReader reader, DataPublisher publisher)
{
    sensors_.emplace_back(sensor, reader, publisher);
}

} // namespace platform::sensors
