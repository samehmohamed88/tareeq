#pragma once

#include "platform/errors/Errors.hpp"
#include "platform/vehicle/wave_rover/WaveRoverNetworkDeviceManager.hpp"
#include "platform/vehicle/wave_rover/WaveRoverUtils.hpp"

#include <future>
#include <memory>
#include <optional>
#include <tuple>
#include <unordered_map>

namespace platform::vehicle::waverover {

template<typename IMUDataType>
class WaveRoverIMUController
{
public:
    WaveRoverIMUController()= default;
//        : retrieveIMUDataTypeCommand_{utils::CommandFactory::createRetrieveImuDataCommand()} {};

    std::tuple<errors::IMUError, std::optional<IMUDataType>> readData();

private:
//    utils::WaveRoverCommand retrieveIMUDataTypeCommand_;
    WaveRoverNetworkDeviceManager networkDeviceManager_;

};

template<typename IMUDataType>
std::tuple<errors::IMUError, std::optional<IMUDataType>> WaveRoverIMUController<IMUDataType>::readData()
{
    try {
        auto response = networkDeviceManager_.read();
        if (response.has_value()) {
            // TODO : validate response and make sure IMU data is sensible
            IMUDataType imuData = utils::jsonToIMUData(response.value());
            return {errors::IMUError::NONE, imuData};
        }
    } catch (const std::exception& e) {
        // In case of any exception during setup, return immediately with an error
        std::cerr << "WaveRoverIMUController::readData : Error occurred " << e.what() << std::endl;
    }
    return {errors::IMUError::FAILURE, std::nullopt};
}

} // namespace platform::vehicle::waverover
