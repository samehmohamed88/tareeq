#pragma once

#include "platform/vehicles/wave_rover/WaveRoverDevicesManager.hpp"
#include "platform/logging/Logger.hpp"

#include <memory>
#include <string>
#include <utility>

namespace platform::vehicles::waverover {

class WaveRoverImuReader : private WaveRoverDevicesManager
{
public:
    WaveRoverImuReader(std::shared_ptr<io::BoostSerialDeviceManager> serialDeviceManager,
                       std::string serialDeviceName = "/dev/ttyUSB0");

    std::tuple<platform::io::Status, std::optional<std::string>> operator()() const;
};

} // namespace platform::vehicles::waverover
