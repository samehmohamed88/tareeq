#pragma once

#include "platform/io/BoostSerialDeviceManager.hpp"
#include "platform/logging/Logger.hpp"

#include <fmt/format.h>

#include <memory>
#include <string>
#include <utility>

namespace platform::vehicles::waverover {

/// @brief Manages device communications for Wave Share UGV01 Platform.
/// WAVE ROVER Flexible And Expandable 4WD Mobile Robot.
/// https://www.waveshare.com/wiki/UGV01
class WaveRoverDevicesManager
{
protected:
    /// @brief Nested class to hold static command generation methods for device communication.
    class CommandConstants
    {
    public:
        /// @brief Generates a command string for speed control.
        /// @param velocityLeft Left wheel velocity in meters per second.
        /// @param velocityRight Right wheel velocity in meters per second.
        /// @return Command string formatted for the device.
        static std::string getSpeedCtrlCommand(double velocityLeft, double velocityRight)
        {
            return fmt::format(R"({{"T":1,"L":{},"R":{}}})"
                               "\n",
                               velocityLeft,
                               velocityRight);
        }

        /// @brief Generates a command string for ROS control.
        /// @param linearVelocity Linear velocity in meters per second.
        /// @param angularVelocity Angular velocity in radians per second.
        /// @return Command string formatted for the device.
        static std::string getROSCtrlCommand(double linearVelocity, double angularVelocity)
        {
            return fmt::format(R"({{"T":13,"X":{},"Z":{}}})"
                               "\n",
                               linearVelocity,
                               angularVelocity);
        }

        /// @brief Generates a command string to retrieve IMU data.
        /// @return Command string formatted for the device.
        static std::string getImuCommand()
        {
            return R"({"T":126})"
                   "\n";
        }
    };

public:
    /// @brief Constructs a new WaveRoverDevicesManager with specified serial device manager and device name.
    /// @param serialDeviceManager Shared pointer to the BoostSerialDeviceManager to handle serial communication.
    /// @param serialDeviceName Name or path of the serial device, default is "/dev/ttyUSB0".
    WaveRoverDevicesManager(std::shared_ptr<io::BoostSerialDeviceManager> serialDeviceManager,
                            std::string serialDeviceName = "/dev/ttyUSB0");

    /// @brief Reads IMU data from the device.
    /// @return A tuple containing the status of the operation and an optional string with the IMU data if successful.
    //    std::tuple<platform::io::Status, std::optional<std::string>> readImuData();

protected:
    logging::Logger logger_;             ///< Logger instance for logging events or errors.
    const std::string serialDeviceName_; ///< Device name or path used for serial communication.
    std::shared_ptr<io::BoostSerialDeviceManager> serialDeviceManager_; ///< Manages the serial communication.
};

} // namespace platform::vehicles::waverover
