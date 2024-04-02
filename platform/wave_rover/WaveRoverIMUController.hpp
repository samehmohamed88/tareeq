#pragma once

#include "platform/sensors/imu/IMUData.hpp"
#include "platform/sensors/imu/IMUDeviceController.hpp"
#include "platform/wave_rover/WaveRoverUtils.hpp"
#include "platform/errors/Errors.hpp"

#include <future>
#include <optional>
#include <memory>
#include <unordered_map>

namespace platform::waverover {


template<typename DeviceManager, typename ILogger>
class WaveRoverIMUController : public sensors::imu::IMUDeviceController<errors::IMUError, DeviceManager, ILogger>
{
public:
    WaveRoverIMUController(std::shared_ptr<DeviceManager> deviceManager, std::shared_ptr<const ILogger> logger);

    std::variant<bool, errors::IMUError> initialize() override;

    std::variant<bool, errors::IMUError> close() override;

    std::tuple<errors::IMUError, std::optional<sensors::imu::IMUData>> readData() override;

    std::variant<bool, errors::IMUError> calibrate() override;

    std::variant<bool, errors::IMUError> checkStatus() override;

    std::variant<bool, errors::IMUError> reset() override;

private:
    utils::WaveRoverCommand retrieveIMUDataCommand_;
};

#include <future>
#include <tuple>

template<typename DeviceManager, typename ILogger>
std::tuple<errors::IMUError, std::optional<sensors::imu::IMUData>> WaveRoverIMUController<DeviceManager, ILogger>::readData()
{
    const std::string command = retrieveIMUDataCommand_.toJsonString();
    this->logger_->logInfo("WaveRoverIMUController::readData creating json command " + command);

    try {
        auto response = this->deviceManager_->read(command);
        if (response.has_value()) {
            // TODO : validate response and make sure IMU data is sensible
            return {errors::IMUError::NONE, utils::jsonToIMUData(response.value())};
        }
    } catch (const std::exception& e) {
        // In case of any exception during setup, return immediately with an error

    }
    return {errors::IMUError::FAILURE, std::nullopt};
}


template<typename DeviceManager, typename ILogger>
WaveRoverIMUController<DeviceManager, ILogger>::WaveRoverIMUController(std::shared_ptr<DeviceManager> deviceManager,
                                                                       std::shared_ptr<const ILogger> logger)
    : sensors::imu::IMUDeviceController<errors::IMUError, DeviceManager, ILogger>(deviceManager, logger)
    , retrieveIMUDataCommand_{utils::CommandFactory::createRetrieveImuDataCommand()}
{}

template<typename DeviceManager, typename ILogger>
std::variant<bool, errors::IMUError> WaveRoverIMUController<DeviceManager, ILogger>::close()
{
    return {};
}

template<typename DeviceManager, typename ILogger>
std::variant<bool, errors::IMUError> WaveRoverIMUController<DeviceManager, ILogger>::initialize()
{
    return {};
}

template<typename DeviceManager, typename ILogger>
std::variant<bool, errors::IMUError> WaveRoverIMUController<DeviceManager, ILogger>::reset()
{
    return {};
}


template<typename DeviceManager, typename ILogger>
std::variant<bool, errors::IMUError> WaveRoverIMUController<DeviceManager, ILogger>::calibrate()
{
    return {};
}

template<typename DeviceManager, typename ILogger>
std::variant<bool, errors::IMUError> WaveRoverIMUController<DeviceManager, ILogger>::checkStatus()
{
    return {};
}


} // namespace platform::sensors::imu
