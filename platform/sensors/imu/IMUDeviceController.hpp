#pragma once

#include "platform/devices/DeviceInterface.hpp"

#include <tuple>

namespace platform::sensors::imu {

struct IMUData {
    // IMU data fields like acceleration, gyroscope, magnetometer, etc.
};

template<typename Error, typename DeviceManager, typename ILogger>
class IMUDeviceController : public devices::DeviceInterface<Error, ILogger>
{
public:
    IMUDeviceController(std::shared_ptr<DeviceManager> deviceManager, std::shared_ptr<const ILogger> logger);

    virtual Error initialize() = 0;

    virtual std::tuple<Error, IMUData> readData() = 0;

    virtual Error calibrate() = 0;

    virtual Error checkStatus() = 0;

    virtual Error reset() = 0;


protected:
    std::shared_ptr<DeviceManager> deviceManager_;
};

template<typename Error, typename DeviceManager, typename ILogger>
IMUDeviceController<Error, DeviceManager, ILogger>::IMUDeviceController(std::shared_ptr<DeviceManager> deviceManager,
                                                                        std::shared_ptr<const ILogger> logger)
    : devices::DeviceInterface<Error, ILogger>(logger)
    , deviceManager_{deviceManager}
{}

} // namespace platforms::sensors::imu
