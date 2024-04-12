
#include "platform/vehicles/wave_rover/WaveRoverImuReader.hpp"

#include "platform/io/BoostSerialDeviceManager.hpp"
#include "platform/vehicles/wave_rover/WaveRoverDevicesManager.hpp"

#include <string>
#include <tuple>
#include <utility>

namespace platform::vehicles::waverover {

WaveRoverImuReader::WaveRoverImuReader(std::shared_ptr<io::BoostSerialDeviceManager> serialDeviceManager,
                                       std::string serialDeviceName)
    : WaveRoverDevicesManager(std::move(serialDeviceManager), std::move(serialDeviceName))
{}

std::tuple<platform::io::Status, std::optional<std::string>> WaveRoverImuReader::operator()() const
{
    logger_.logDebug("Reading IMU Sensor Data from ", serialDeviceName_);
    auto [status, data] = serialDeviceManager_->readFromDevice(serialDeviceName_, CommandConstants::getImuCommand());
    std::cout << status.toString() << " " << *data << std::endl;
    return {status, data};
}
} // namespace platform::vehicles::waverover
