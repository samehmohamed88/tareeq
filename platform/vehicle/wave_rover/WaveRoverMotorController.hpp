#pragma once

#include "platform/errors/Errors.hpp"
#include "platform/motors/MotorController.hpp"
#include "platform/vehicle/config/VehicleConfig.hpp"
#include "platform/vehicle/wave_rover/WaveRoverUtils.hpp"

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
    const double maxLinearSpeed_;
    const double maxAngularSpeed_;
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

    auto command = speedControlCommand_.toJsonString();
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
std::variant<bool, errors::MotorError> WaveRoverMotorController<DeviceManager, ILogger>::setMotorPwm(int leftMotorPwm,
                                                                                                     int rightMotorPwm)
{
    this->logger_->logInfo("WaveRoverMotorController::setMotorPwm " + std::to_string(leftMotorPwm) + " " +
                           std::to_string(rightMotorPwm));
    speedControlCommand_.updateParameter("L", leftMotorPwm);
    speedControlCommand_.updateParameter("R", rightMotorPwm);

    auto command = std::string("/js?json=") + speedControlCommand_.toJsonString();
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
    this->logger_->logInfo("WaveRoverMotorController::setDifferentialDrivePWM " + std::to_string(linearVelocity) + " " +
                           std::to_string(angularVelocity));

    const int maxPWM = 255;
    const double maxLinearSpeed = this->maxLinearSpeed_; // maximum linear speed of the robot
    const double maxAngularSpeed = this->maxAngularSpeed_; // maximum angular speed of the robot

    // Normalize linear and angular velocities
    linearVelocity = (linearVelocity / maxLinearSpeed) * maxPWM;
    angularVelocity = (angularVelocity / maxAngularSpeed) * maxPWM;

//    this->logger_->logInfo("WaveRoverMotorController::setDifferentialDrivePWM NORMALIZED  >>> " + std::to_string(linearVelocity) + " " +
//                           std::to_string(angularVelocity));
//
//    this->logger_->logInfo("WaveRoverMotorController::setDifferentialDrivePWM  >>> " + std::to_string(wheelSeparationCached_) + " " +
//                           std::to_string(wheelRadiusCached_));

    // Compute wheel speeds based on normalized velocities
    double leftWheelSpeed = ((linearVelocity - angularVelocity) * (wheelSeparationCached_ / 2)) / wheelRadiusCached_;
    double rightWheelSpeed = ((linearVelocity + angularVelocity) * (wheelSeparationCached_ / 2)) / wheelRadiusCached_;


//    this->logger_->logInfo("WaveRoverMotorController::setDifferentialDrivePWM leftWheelSpeed, "
//                           "rightWheelSpeed: " +
//                           std::to_string(leftWheelSpeed) + ", " + std::to_string(rightWheelSpeed));

    // Apply scaling to maintain the ratio if any wheel speed exceeds maxPWM
    double maxWheelSpeed = std::max(std::abs(leftWheelSpeed), std::abs(rightWheelSpeed));
    if (maxWheelSpeed > maxPWM) {
        leftWheelSpeed *= maxPWM / maxWheelSpeed;
        rightWheelSpeed *= maxPWM / maxWheelSpeed;
    }

//    this->logger_->logInfo("WaveRoverMotorController::setDifferentialDrivePWM (adjusted) leftWheelSpeed, "
//                           "rightWheelSpeed: " +
//                           std::to_string(leftWheelSpeed) + ", " + std::to_string(rightWheelSpeed));

    // Threshold the PWM values to account for motor deadband
    int leftPwm = std::abs(leftWheelSpeed) < this->vehicleConfig_.getMotorConfig().getPwmThreshold() * maxPWM
                      ? 0
                      : static_cast<int>(leftWheelSpeed);
    int rightPwm = std::abs(rightWheelSpeed) < this->vehicleConfig_.getMotorConfig().getPwmThreshold() * maxPWM
                       ? 0
                       : static_cast<int>(rightWheelSpeed);

    return this->setMotorPwm(leftPwm, rightPwm);
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
    , wheelRadiusCached_{this->vehicleConfig_.getChassisConfig().getRearWheelRadius()}
    , wheelSeparationCached_{this->vehicleConfig_.getChassisConfig().getRearWheelSeparation()}
    , maxLinearSpeed_{1.0}
    , maxAngularSpeed_{1.0}
{}

} // namespace platform::vehicle::waverover
