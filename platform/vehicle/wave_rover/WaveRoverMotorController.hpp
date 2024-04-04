#pragma once

#include "platform/motors/MotorController.hpp"
#include "platform/vehicle/config/VehicleConfig.hpp"
#include "platform/vehicle/wave_rover/WaveRoverUtils.hpp"
#include "platform/errors/Errors.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <mutex>
#include <unordered_map>

namespace platform::vehicle::waverover {

template<typename DeviceManager, typename ILogger>
class WaveRoverMotorController : public motors::MotorController<errors::MotorError, DeviceManager, ILogger>
{
public:
    WaveRoverMotorController(std::shared_ptr<DeviceManager> deviceManager,
                             std::shared_ptr<const ILogger> logger,
                             const vehicle::VehicleConfig& vehicleConfig);

    std::variant<bool, errors::MotorError> initialize() override;

    std::variant<bool, errors::MotorError> stop() override;

    std::variant<bool, errors::MotorError> setVelocity(double linearVelocity, double angularVelocity) override;

    std::variant<bool, errors::MotorError> close() override;

private:
    std::variant<bool, errors::MotorError> setDifferentialDrivePWM(double linearVelocity, double angularVelocity);
    std::variant<bool, errors::MotorError> setMotorPwm(int leftMotorPwm, int rightMotorPwm) override;

    utils::WaveRoverCommand speedControlCommand_;
    const double wheelRadiusCached_;
    const double wheelSeparationCached_;
    std::mutex setVehicleVelocity_;
};

template<typename DeviceManager, typename ILogger>
std::variant<bool, errors::MotorError> WaveRoverMotorController<DeviceManager, ILogger>::close()
{
    return {};
}

template<typename DeviceManager, typename ILogger>
std::variant<bool, errors::MotorError> WaveRoverMotorController<DeviceManager, ILogger>::stop()
{
    this->logger_->logInfo("WaveRoverMotorController::stop()");
    speedControlCommand_.updateParameter("L", 0);
    speedControlCommand_.updateParameter("R", 0);

    const std::string& command = speedControlCommand_.toJsonString();
    this->logger_->logInfo("WaveRoverMotorController::stop() creating json output " + command);
    try {
        this->deviceManager_->write(command);
    } catch (const std::exception& e) {
        return errors::MotorError::COMMUNICATION_ERROR;
    }

    return true;
}

template<typename DeviceManager, typename ILogger>
std::variant<bool, errors::MotorError> WaveRoverMotorController<DeviceManager, ILogger>::initialize()
{
    return {};
}

template<typename DeviceManager, typename ILogger>
std::variant<bool, errors::MotorError> WaveRoverMotorController<DeviceManager, ILogger>::setMotorPwm(
    int leftMotorPwm,
    int rightMotorPwm)
{
    this->logger_->logInfo("WaveRoverMotorController::setMotorPwm " + std::to_string(leftMotorPwm) + " " +
                           std::to_string(rightMotorPwm));
    speedControlCommand_.updateParameter("L", leftMotorPwm);
    speedControlCommand_.updateParameter("R", rightMotorPwm);

    const std::string& command = speedControlCommand_.toJsonString();
    this->logger_->logInfo("WaveRoverMotorController::setMotorPwm creating json output " + command);

    try {
        this->deviceManager_->write(command);
    } catch (const std::exception& e) {
        return errors::MotorError::COMMUNICATION_ERROR;
    }
    return true;
}

template<typename DeviceManager, typename ILogger>
std::variant<bool, errors::MotorError> WaveRoverMotorController<DeviceManager, ILogger>::setDifferentialDrivePWM(
    double linearVelocity,
    double angularVelocity)
{
    // max PWM is 255 (8 bit digit)
    const int maxPWM = 255;

    // TODO: maybe validate input and if invalid use previous values
    // TODO: keep a history of 3 most recent velocity values?
    double leftWheelSpeed = ((2 * linearVelocity) - (angularVelocity * wheelSeparationCached_)) / (wheelRadiusCached_);
    double rightWheelSpeed = ((2 * linearVelocity) + (angularVelocity * wheelSeparationCached_)) / (wheelRadiusCached_);

    double maxWheelSpeed = std::max(std::abs(leftWheelSpeed), std::abs(rightWheelSpeed));
    if (maxWheelSpeed > maxPWM) {
        leftWheelSpeed *= maxPWM / maxWheelSpeed;
        rightWheelSpeed *= maxPWM / maxWheelSpeed;
    }

    // we use this to threshold PWM values to above 50 or 0
    // DC gear motors have poor low-speed characteristics
    // the motor may not rotate when the absolute value of the PWM is too small
    return this->setMotorPwm(
        static_cast<int>(std::abs(leftWheelSpeed) < this->vehicleConfig_.getMotorConfig().getPwmThreshold() * maxPWM
                             ? 0
                             : leftWheelSpeed),
        static_cast<int>(std::abs(rightWheelSpeed) < this->vehicleConfig_.getMotorConfig().getPwmThreshold() * maxPWM
                             ? 0
                             : rightWheelSpeed));
}

template<typename DeviceManager, typename ILogger>
std::variant<bool, errors::MotorError> WaveRoverMotorController<DeviceManager, ILogger>::setVelocity(
    double linearVelocity,
    double angularVelocity)
{
    std::lock_guard<std::mutex> lock(setVehicleVelocity_);
    return setDifferentialDrivePWM(linearVelocity, angularVelocity);
}

template<typename DeviceManager, typename ILogger>
WaveRoverMotorController<DeviceManager, ILogger>::WaveRoverMotorController(std::shared_ptr<DeviceManager> deviceManager,
                                                                           std::shared_ptr<const ILogger> logger,
                                                                           const vehicle::VehicleConfig& vehicleConfig)
    : motors::MotorController<errors::MotorError, DeviceManager, ILogger>(deviceManager, logger, vehicleConfig)
    , speedControlCommand_{utils::CommandFactory::createSpeedControlCommand(0, 0)}
    , wheelRadiusCached_{2 * this->vehicleConfig_.getChassisConfig().getRearWheelRadius()}
    , wheelSeparationCached_{this->vehicleConfig_.getChassisConfig().getRearWheelSeparation()}
{}

} // namespace platform::vehicle::waverover
