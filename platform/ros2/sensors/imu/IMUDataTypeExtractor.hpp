#pragma once

#include "platform/vehicle/wave_rover/WaveRoverIMUController.hpp"
#include "platform/zed/ZedIMUController.hpp"

namespace platform::sensors::imu {
// Define traits to extract IMU data type from the controller types
template<typename T>
struct IMUDataTypeExtractor;

template<typename T>
struct IMUDataTypeExtractor<platform::zed::ZedIMUController<T>>
{
    using type = T;
};

template<typename T>
struct IMUDataTypeExtractor<vehicle::waverover::WaveRoverIMUController<T>>
{
    using type = T;
};
} // namespace platform::sensors::imu
