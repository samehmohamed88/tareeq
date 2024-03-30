#pragma once

#include <boost/asio.hpp>

#include <cstddef>

namespace platform::io {

class AsioOperationsImpl : public IAsioOperations {
public:
    void write(boost::asio::serial_port& serial, const boost::asio::const_buffer& buffer) override {
        boost::asio::write(serial, buffer);
    }

    std::size_t read(boost::asio::serial_port& serial, boost::asio::mutable_buffer buffer) override {
        return boost::asio::read(serial, buffer);
    }
};

} // namespace platform::io
