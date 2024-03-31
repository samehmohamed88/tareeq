#pragma once

#include "platform/io/IAsioOperations.hpp"

#include <boost/asio.hpp>

#include <cstddef>

namespace platform::io {

class AsioOperationsImpl  {
public:
    void write(boost::asio::serial_port& serial, const boost::asio::const_buffer& buffer) {
        boost::asio::write(serial, buffer);
    }

    std::size_t read(boost::asio::serial_port& serial, boost::asio::mutable_buffer buffer) {
        return boost::asio::read(serial, buffer);
    }
};

} // namespace platform::io
