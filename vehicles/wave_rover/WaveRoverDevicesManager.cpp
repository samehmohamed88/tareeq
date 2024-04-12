#include "platform/vehicles/wave_rover/WaveRoverDevicesManager.hpp"

#include "platform/io/BoostSerialDeviceManager.hpp"

#include <string>
#include <utility>

namespace platform::vehicles::waverover {
WaveRoverDevicesManager::WaveRoverDevicesManager(std::shared_ptr<io::BoostSerialDeviceManager> serialDeviceManager,
                                                 std::string serialDeviceName)
    : logger_{"WaveRoverDevicesManager"}
    , serialDeviceName_{std::move(serialDeviceName)}
    , serialDeviceManager_{std::move(serialDeviceManager)}
{
    auto status = serialDeviceManager_->createDevice(serialDeviceName_, 115200);
    if (status.isSuccess()) {
        // TODO: what happens if the createDevice didn't work?? try again? exit?
        logger_.logDebug("Successfully opened serial device {}", serialDeviceName_);
    }
}

} // namespace platform::vehicles::waverover
