#pragma once

#include "platform/motors/MotorController.hpp"
#include "platform/wave_rover/WaveRoverUtils.hpp"

#include <exception>
#include <unordered_map>

namespace platform::waverover {

enum class MotorControllerErrors
{
    None,
    NotInitialized,
    SerialPortError,
    InvalidParameter,
    CommunicationError,
};

template<typename DeviceManager,
         typename ILogger>
class WaveRoverMotorController : public motors::MotorController<MotorControllerErrors, DeviceManager, ILogger>
{
public:
    WaveRoverMotorController(std::shared_ptr<DeviceManager> deviceManager, std::shared_ptr<const ILogger> logger);

    std::variant<bool, MotorControllerErrors> initialize() override;

    std::variant<bool, MotorControllerErrors> stop() override;

    std::variant<bool, MotorControllerErrors> setWheelSpeeds(double leftWheelSpeed, double rightWheelSpeed) override;

    std::variant<bool, MotorControllerErrors> setMotorPwm(int leftMotorPwm, int rightMotorPwm) override;

    std::variant<bool, MotorControllerErrors> close() override;

private:
    utils::WaveRoverCommand speedControlCommand_;
};

template<typename DeviceManager, typename ILogger>
std::variant<bool, MotorControllerErrors> WaveRoverMotorController<DeviceManager, ILogger>::close()
{
    return {};
}

template<typename DeviceManager, typename ILogger>
std::variant<bool, MotorControllerErrors> WaveRoverMotorController<DeviceManager, ILogger>::stop()
{
    this->logger_->logInfo("WaveRoverMotorController::stop()");
    speedControlCommand_.updateParameter("L", 0);
    speedControlCommand_.updateParameter("R", 0);

    const std::string& command = speedControlCommand_.toJsonString();
    this->logger_->logInfo("WaveRoverMotorController::stop() creating json output " + command);
    try {
        this->deviceManager_->write(command);
    } catch (const std::exception& e) {
        return MotorControllerErrors::CommunicationError;
    }

    return true;
}

template<typename DeviceManager, typename ILogger>
std::variant<bool, MotorControllerErrors> WaveRoverMotorController<DeviceManager, ILogger>::initialize()
{
    return {};
}

template<typename DeviceManager, typename ILogger>
std::variant<bool, MotorControllerErrors> WaveRoverMotorController<DeviceManager, ILogger>::setMotorPwm(
    int leftMotorPwm,
    int rightMotorPwm)
{
    this->logger_->logInfo("WaveRoverMotorController::setWheelSpeeds " + std::to_string(leftMotorPwm) + " " + std::to_string(rightMotorPwm));
    speedControlCommand_.updateParameter("L", leftMotorPwm);
    speedControlCommand_.updateParameter("R", rightMotorPwm);

//    const std::string& command = speedControlCommand_.toJsonString();
//    this->logger_->logInfo("WaveRoverMotorController::setWheelSpeeds creating json output " + command);
//
//    try {
//        this->deviceManager_->write(command);
//    } catch (const std::exception& e) {
//        return MotorControllerErrors::CommunicationError;
//    }

    return true;
}


template<typename DeviceManager, typename ILogger>
std::variant<bool, MotorControllerErrors> WaveRoverMotorController<DeviceManager, ILogger>::setWheelSpeeds(
    double leftWheelSpeed,
    double rightWheelSpeed)
{
    this->logger_->logInfo("WaveRoverMotorController::setWheelSpeeds " + std::to_string(leftWheelSpeed) + " " + std::to_string(rightWheelSpeed));
    speedControlCommand_.updateParameter("L", leftWheelSpeed);
    speedControlCommand_.updateParameter("R", rightWheelSpeed);

    const std::string& command = speedControlCommand_.toJsonString();
    this->logger_->logInfo("WaveRoverMotorController::setWheelSpeeds creating json output " + command);

    try {
        this->deviceManager_->write(command);
    } catch (const std::exception& e) {
        return MotorControllerErrors::CommunicationError;
    }

    return true;
}

template<typename DeviceManager, typename ILogger>
WaveRoverMotorController<DeviceManager, ILogger>::WaveRoverMotorController(std::shared_ptr<DeviceManager> deviceManager,
                                                                           std::shared_ptr<const ILogger> logger)
    : motors::MotorController<MotorControllerErrors, DeviceManager, ILogger>(deviceManager, logger)
    , speedControlCommand_{utils::CommandFactory::createSpeedControlCommand(0, 0)}
{}


} // namespace platform::motors
