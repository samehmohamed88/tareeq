#pragma once

#include <string>
#include <memory>
#include <exception>

namespace platform::devices {

template<
    typename SerialPortImpl,
    typename ILogger>
class SerialDeviceManager
{
public:
    SerialDeviceManager(std::shared_ptr<SerialPortImpl> serialPort, std::shared_ptr<ILogger> logger)
        : serialPort_{serialPort}
        , logger_{logger}
    {}

    void write(const std::string& data) {
        std::lock_guard<std::mutex> lock(write_mutex_);
        try {
            serialPort_->write(data);
        } catch (const std::exception& e) {

        }
    }

    std::string read() { return serialPort_->read(); }

private:
    std::shared_ptr<SerialPortImpl> serialPort_;
    std::shared_ptr<ILogger> logger_;

    std::mutex read_mutex_;  // Mutex to synchronize access to the read method
    std::mutex write_mutex_; // Mutex to synchronize access to the write method
};

} // namespace platform::devices
