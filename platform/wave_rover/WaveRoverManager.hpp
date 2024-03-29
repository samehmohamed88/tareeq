#pragma once

#include <memory>
#include <utility>

namespace platform::waverover {

template<typename MotorController, typename IMUDeviceController, typename OLEDDeviceController, typename ILogger>
class WaveRoverManager
{
public:
    enum class Error
    {
        None,
        NotInitialized,
        SerialPortError,
        InvalidParameter,
        CommunicationError,
    };

    enum class CommandSet
    {
        CMD_SPEED_CTRL = 1,
        CMD_PWM_INPUT = 11,
        CMD_OLED_SCREEN_CONTROL = 3,
        CMD_OLED_SCREEN_RESTORE = -3,

    };

public:
    WaveRoverManager(MotorController&& motorController,
                     IMUDeviceController&& imuDeviceController,
                     OLEDDeviceController&& oledDeviceController,
                     std::shared_ptr<ILogger> logger);

private:
    MotorController motorController_;
    IMUDeviceController imuDeviceController_;
    OLEDDeviceController oledDeviceController_;
    std::shared_ptr<ILogger> logger_;
};

template<typename MotorController, typename IMUDeviceController, typename OLEDDeviceController, typename ILogger>
WaveRoverManager<MotorController, IMUDeviceController, OLEDDeviceController, ILogger>::WaveRoverManager(
    MotorController&& motorController,
    IMUDeviceController&& imuDeviceController,
    OLEDDeviceController&& oledDeviceController,
    std::shared_ptr<ILogger> logger)
    : motorController_{std::move(motorController)}
    , imuDeviceController_{std::move(imuDeviceController)}
    , oledDeviceController_{std::move(oledDeviceController)}
    , logger_{logger}
{}

} // namespace platform::waverover
