#pragma once

#include <boost/asio.hpp>

#include <cstddef>

namespace platform::io {

class IAsioOperations {
public:
    virtual ~IAsioOperations() = default;
    virtual void write(boost::asio::serial_port& serial, const boost::asio::const_buffer& buffer) = 0;
    virtual std::size_t read(boost::asio::serial_port& serial, boost::asio::mutable_buffer buffer) = 0;
};

} // namespace platform::io
