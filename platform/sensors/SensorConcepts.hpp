#pragma once

#include "platform/io/Status.hpp"

#include <concepts>
#include <iostream>
#include <optional>

namespace platform::sensors {

// A generic Sensor concept that can return any data type.
template<typename Sensor, typename SensorData>
concept ReadableSensor = requires(Sensor sensor) {
    {
        sensor.read()
    };
};

template<typename Sensor>
concept RequiresInitialization = requires(Sensor sensor) {
    {
        sensor.initialize()
    } -> std::same_as<platform::io::Status>;
};

} // namespace platform::sensors
