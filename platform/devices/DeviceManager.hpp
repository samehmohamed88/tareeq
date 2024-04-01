#pragma once

#include <nlohmann/json.hpp>
#include <string>
#include <memory>
#include <exception>
#include <optional>

namespace platform::devices {

template<
    typename DeviceImpl,
    typename ILogger>
class DeviceManager
{
public:
    DeviceManager(std::shared_ptr<DeviceImpl> device, std::shared_ptr<ILogger> logger)
        : device_{device}
        , logger_{logger}
    {}

    // TODO: right now this does not offer any real value
    virtual void write(const std::string& data) {
        std::lock_guard<std::mutex> lock(write_mutex_);
        try {
            device_->write(data);
        } catch (const std::exception& e) {
            // TODO: make this meaning full
        }
    }

    // TODO: potentially add another read method that takes in a callback std::function
    // TODO: right now this does not offer any real value
    virtual std::optional<std::string> read(const std::string& request) {
        std::lock_guard<std::mutex> lock(read_mutex_);

        try {
            return device_->read(request);
        } catch (const std::exception& e) {
            // TODO: make this meaning full
        }
        return std::nullopt;
    }

protected:
    std::shared_ptr<DeviceImpl> device_;
    std::shared_ptr<ILogger> logger_;

    std::mutex read_mutex_;  // Mutex to synchronize access to the read method
    std::mutex write_mutex_; // Mutex to synchronize access to the write method
};

} // namespace platform::devices
