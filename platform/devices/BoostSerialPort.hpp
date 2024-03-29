#pragma once

#include <boost/asio.hpp>

#include <array>
#include <string>

namespace av::devices {

class BoostSerialPort : public ISerialPort {
public:
    BoostSerialPort(const std::string& port, unsigned int baud_rate)
        : io_(), serial_(io_, port) {
        serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    }

    void write(const std::string& data) override {
        boost::asio::write(serial_, boost::asio::buffer(data));
    }

    std::string read() override {
        std::array<char, 128> buf{};
        boost::asio::read(serial_, boost::asio::buffer(buf), boost::asio::transfer_at_least(1));
        return std::string(buf.data());
    }

private:
    boost::asio::io_context io_;
    boost::asio::serial_port serial_;
};


}; // namespace av::devices
