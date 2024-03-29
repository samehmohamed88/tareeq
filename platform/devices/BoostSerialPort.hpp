#pragma once

#include "platform/devices/ISerialPort.hpp"

#include <boost/asio.hpp>

#include <mutex>
#include <thread>
#include <array>
#include <string>

namespace platform::devices {

    class BoostSerialPort : public ISerialPort {
    public:
        BoostSerialPort(const std::string &port, uint32_t baud_rate=115200);
        ~BoostSerialPort() override;

        void write(const std::string &data) override;

        void read(const ReadCallback& callback) override;

    private:
        void stop();
        boost::asio::io_context io_context_;
        boost::asio::serial_port serial_;
        std::thread read_thread_;
        std::mutex read_mutex_;  // Mutex to synchronize access to the read method
    };

}; // namespace platform::devices
