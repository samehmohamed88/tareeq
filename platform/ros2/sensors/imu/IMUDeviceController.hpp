#pragma once

#include "platform/sensors/imu/IMUData.hpp"
#include "platform/devices/DeviceInterface.hpp"

#include <tuple>
#include <optional>

namespace platform::sensors::imu {

template<typename Error, typename DeviceManager, typename ILogger>
class IMUDeviceController : public devices::DeviceInterface<Error, ILogger>
{
public:
    IMUDeviceController(std::shared_ptr<DeviceManager> deviceManager, std::shared_ptr<const ILogger> logger);

    virtual std::tuple<Error, std::optional<IMUData>> readData() = 0;

    virtual std::variant<bool, Error> calibrate() = 0;

    virtual std::variant<bool, Error> checkStatus() = 0;

    virtual std::variant<bool, Error> reset() = 0;

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
