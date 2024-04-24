#pragma once

#include <iostream>
#include <string>

namespace platform::io {

/// @enum STATUS
/// @brief Represents the basic status of an operation.
///
/// Enumerates the possible high-level outcomes of an operation,
/// distinguishing between successful completion and errors.
enum class STATUS
{
    SUCCESS, ///< Operation succeeded without any error.
    ERROR    ///< Operation failed, specifics provided by ERROR enum.
};

/// @enum ERROR
/// @brief Defines specific error types for operations that result in errors.
///
/// Enumerates detailed error types that describe the nature of the failure
/// when the status is ERROR.
enum class ERROR
{
    NONE,                ///< No error occurred.
    TIMEOUT,             ///< Operation timed out.
    COMMUNICATION_ERROR, ///< Communication failure during operation.
    SENSOR_FAILURE,       ///< Sensor malfunction or failure detected.
    ERROR_CREATING_DEVICE,
    ERROR_OPENING_DEVICE
};

/// @class Status
/// @brief Represents the outcome of an operation, with detailed error information if applicable.
///
/// This class encapsulates both a basic status (success or error) and,
/// in the case of errors, provides detailed information about the type of error.
class Status
{
public:
    /// @brief Constructor for Status.
    /// @param bs The basic status (SUCCESS or ERROR).
    /// @param et The specific error type if an error has occurred.
    Status(STATUS bs = STATUS::SUCCESS, ERROR et = ERROR::NONE, std::string errorMessage = std::string())
        : basicStatus_{bs}
        , errorType_{et}
        , errorMessage_{std::move(errorMessage)}
    {}

    /// @brief Checks if the status indicates success.
    /// @return True if the status is SUCCESS, false otherwise.
    bool isSuccess() const { return basicStatus_ == STATUS::SUCCESS; }

    /// @brief Checks if the status indicates an error.
    /// @return True if the status is ERROR, false otherwise.
    bool isError() const { return basicStatus_ == STATUS::ERROR; }

    /// @brief Checks if the error type matches the specified type.
    /// @param et The error type to compare against.
    /// @return True if the error type matches, false otherwise.
    bool isErrorType(ERROR et) const { return errorType_ == et; }

    /// @brief Converts the status to a string representation.
    /// @return A string that describes the status or error in detail.
    std::string toString() const
    {
        if (isSuccess()) {
            return "Success";
        }
        switch (errorType_) {
            case ERROR::TIMEOUT:
                return "Timeout Error " + errorMessage_;
            case ERROR::COMMUNICATION_ERROR:
                return "Communication Error " + errorMessage_;
            case ERROR::SENSOR_FAILURE:
                return "Sensor Failure " + errorMessage_;
            case ERROR::ERROR_CREATING_DEVICE:
                return "Error Creating Device " + errorMessage_;
            case ERROR::ERROR_OPENING_DEVICE:
                return "Error Opening Device " + errorMessage_;
            default:
                return "Unknown Error" + errorMessage_;
        }
    }

    /// @brief Overloads the output stream operator for Status objects.
    /// @param os The output stream to send the data.
    /// @param status The Status object to output.
    /// @return A reference to the modified output stream.
    /// @details Utilizes the toString method to output the status.
    friend std::ostream& operator<<(std::ostream& os, const Status& status)
    {
        os << status.toString();
        return os;
    }

private:
    STATUS basicStatus_;
    ERROR errorType_;
    std::string errorMessage_;
};
//
// inline std::string toString(MotorError motorControllerErrors) {
//    switch (motorControllerErrors) {
//        case MotorError::NONE: return "NONE";
//        case MotorError::TIMEOUT: return "TIMEOUT";
//        case MotorError::COMMUNICATION_ERROR: return "COMMUNICATION_ERROR";
//        case MotorError::MOTOR_FAILURE: return "MOTOR_FAILURE";
//        default: return "NONE";
//    }
//}
enum class IMUError
{
    NONE,
    FAILURE,
    TIMEOUT,
    UNCALIBRATED,
    SENSOR_FAILURE,
    UNINITIALIZED
};

} // namespace platform::io
