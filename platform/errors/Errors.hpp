#pragma once

#include <string>

namespace platform::errors {

enum class Errors
{
    NONE,
    FAILURE,
    TIMEOUT,
};

enum class MotorError
{
    NONE,
    TIMEOUT,
    COMMUNICATION_ERROR,
    MOTOR_FAILURE,
};

std::string toString(MotorError motorControllerErrors) {
    switch (motorControllerErrors) {
        case MotorError::NONE: return "NONE";
        case MotorError::TIMEOUT: return "TIMEOUT";
        case MotorError::COMMUNICATION_ERROR: return "COMMUNICATION_ERROR";
        case MotorError::MOTOR_FAILURE: return "MOTOR_FAILURE";
    }
}

enum class IMUError {
    NONE,
    FAILURE,
    TIMEOUT,
    UNCALIBRATED,
    SENSOR_FAILURE,
    UNINITIALIZED
};

} // namespace platform::errors
