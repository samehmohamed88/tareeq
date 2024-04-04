#pragma once

#include "platform/devices/DeviceManager.hpp"
#include "platform/vehicle/wave_rover/WaveRoverUtils.hpp"

#include <exception>
#include <memory>
#include <optional>
#include <string>
#include <mutex>

namespace platform::vehicle::waverover {

template<
    typename BoostNetworkIO,
    typename ILogger>
class WaveRoverNetworkDeviceManager : public devices::DeviceManager<BoostNetworkIO, ILogger>
{
public:
    WaveRoverNetworkDeviceManager(std::shared_ptr<BoostNetworkIO> boostIO, std::shared_ptr<ILogger> logger)
        : devices::DeviceManager<BoostNetworkIO, ILogger>(boostIO, logger)
    {}

    virtual void write(const std::string& data) {
        std::lock_guard<std::mutex> lock(this->write_mutex_);
        internalWrite(data);
    }

    virtual std::optional<std::string> read(const std::string& request) {
        std::lock_guard<std::mutex> readLock(this->read_mutex_);
        {
            std::lock_guard<std::mutex> writeLock(this->write_mutex_);
            internalWrite(request);
        }
        try {
            // this is the address to retrieve
            // what was written to memory in above write call
            // Immediately send a follow-up request to /jsfb endpoint to get the response
            this->device_->reconnect();
            return this->device_->read("/jsfb");
        } catch (const std::exception& e) {
            // Handle the exception
            this->logger_->logError("WaveRoverNetworkDeviceManager::read " + std::string(e.what()));
        }
        return std::nullopt;
    }
private:

    void internalWrite(const std::string& data) {
        try {
            this->device_->reconnect();
            this->device_->write("/js?json=" + data);
        } catch (const std::exception& e) {
            // Handle the exception
        }
    }

};

} // namespace platform::devices
