#pragma once

#include "platform/io/IAsioOperations.hpp"

#include <boost/asio.hpp>
#include <cstddef>
#include <iostream>

namespace platform::io {

class AsioOperationsImpl  {
public:
    void printBuffer(const boost::asio::const_buffer& buffer) {
        // Get a pointer to the buffer data
        const char* data = boost::asio::buffer_cast<const char*>(buffer);

        // Get the size of the buffer
        std::size_t size = boost::asio::buffer_size(buffer);

        // Create a string from the buffer data to ensure it handles null termination correctly
        std::string str(data, size);

        // Print the string
        std::cout << str << std::endl;
    }

    void write(boost::asio::serial_port& serial, const boost::asio::const_buffer& buffer) {
        std::cout << "Info: AsioOperationsImpl::write writing buffer ";
        printBuffer(buffer);
        boost::asio::write(serial, buffer);
    }

    std::size_t read(boost::asio::serial_port& serial, boost::asio::mutable_buffer buffer) {
        return boost::asio::read(serial, buffer);
    }
};

} // namespace platform::io
