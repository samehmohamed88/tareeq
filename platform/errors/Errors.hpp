#pragma once

namespace platform::errors {

enum class Errors
{
    NONE,
    FAILURE,
    TIMEOUT,
};

enum class IMUError {
    NONE,
    FAILURE,
    TIMEOUT,
    UNCALIBRATED,
    SENSOR_FAILURE,
    UNINITIALIZED
};

} // namespace platform::errors
