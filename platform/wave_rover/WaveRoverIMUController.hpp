#pragma once

#include "platform/sensors/imu/IMUData.hpp"
#include "platform/sensors/imu/IMUDeviceController.hpp"
#include "platform/wave_rover/WaveRoverUtils.hpp"

#include <future>
#include <optional>
#include <memory>
#include <unordered_map>

namespace platform::waverover {

enum class IMUControllerErrors
{
    None,
    NotInitialized,
    SerialPortError,
    InvalidParameter,
    CommunicationError,
};

template<typename DeviceManager, typename ILogger>
class WaveRoverIMUController : public sensors::imu::IMUDeviceController<IMUControllerErrors, DeviceManager, ILogger>
{
public:
    WaveRoverIMUController(std::shared_ptr<DeviceManager> deviceManager, std::shared_ptr<const ILogger> logger);

    std::variant<bool, IMUControllerErrors> initialize() override;

    std::variant<bool, IMUControllerErrors> close() override;

    std::tuple<IMUControllerErrors, std::optional<sensors::imu::IMUData>> readData() override;

    std::variant<bool, IMUControllerErrors> calibrate() override;

    std::variant<bool, IMUControllerErrors> checkStatus() override;

    std::variant<bool, IMUControllerErrors> reset() override;

private:
    utils::WaveRoverCommand retrieveIMUDataCommand_;
};

#include <future>
#include <tuple>

template<typename DeviceManager, typename ILogger>
std::tuple<IMUControllerErrors, std::optional<sensors::imu::IMUData>> WaveRoverIMUController<DeviceManager, ILogger>::readData()
{
    const std::string command = retrieveIMUDataCommand_.toJsonString();
    this->logger_->logInfo("WaveRoverIMUController::readData creating json command " + command);

    try {
        auto response = this->deviceManager_->read(command);
        if (response.has_value()) {
            // TODO : validate response and make sure IMU data is sensible
            return {IMUControllerErrors::None, utils::jsonToIMUData(response.value())};
        }
    } catch (const std::exception& e) {
        // In case of any exception during setup, return immediately with an error

    }
    return {IMUControllerErrors::CommunicationError, std::nullopt};
}


template<typename DeviceManager, typename ILogger>
WaveRoverIMUController<DeviceManager, ILogger>::WaveRoverIMUController(std::shared_ptr<DeviceManager> deviceManager,
                                                                       std::shared_ptr<const ILogger> logger)
    : sensors::imu::IMUDeviceController<IMUControllerErrors, DeviceManager, ILogger>(deviceManager, logger)
    , retrieveIMUDataCommand_{utils::CommandFactory::createRetrieveImuDataCommand()}
{}

template<typename DeviceManager, typename ILogger>
std::variant<bool, IMUControllerErrors> WaveRoverIMUController<DeviceManager, ILogger>::close()
{
    return {};
}

template<typename DeviceManager, typename ILogger>
std::variant<bool, IMUControllerErrors> WaveRoverIMUController<DeviceManager, ILogger>::initialize()
{
    return {};
}

template<typename DeviceManager, typename ILogger>
std::variant<bool, IMUControllerErrors> WaveRoverIMUController<DeviceManager, ILogger>::reset()
{
    return {};
}


template<typename DeviceManager, typename ILogger>
std::variant<bool, IMUControllerErrors> WaveRoverIMUController<DeviceManager, ILogger>::calibrate()
{
    return {};
}

template<typename DeviceManager, typename ILogger>
std::variant<bool, IMUControllerErrors> WaveRoverIMUController<DeviceManager, ILogger>::checkStatus()
{
    return {};
}


} // namespace platform::sensors::imu
