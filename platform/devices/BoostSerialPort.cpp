#include "platform/devices/BoostSerialPort.hpp"

#include <thread>
#include <iostream>

namespace platform::devices {
    BoostSerialPort::BoostSerialPort(const std::string &port, uint32_t baud_rate)
            : serial_(io_context_, port) {
        serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
        serial_.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
        serial_.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
        serial_.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
        serial_.set_option(boost::asio::serial_port::character_size(8));
    }

    void BoostSerialPort::write(const std::string &data) {
        std::lock_guard<std::mutex> lock(write_mutex_);
        try {
            boost::asio::write(serial_, boost::asio::buffer(data));
        } catch (const boost::system::system_error& e) {
            std::cerr << "Write error: " << e.what() << std::endl;
            // Handle the error appropriately (e.g., retry, log, or propagate the error)
        }
    }

    // TO-DO: use logging instead of std::cout
    void BoostSerialPort::read(const ReadCallback& callback) {
        std::lock_guard<std::mutex> lock(read_mutex_);
        if (read_thread_.joinable()) {
            // Wait for any existing read operation to finish before starting a new one
            read_thread_.join();
        }

        read_thread_ = std::thread([this, &callback]() {
            boost::asio::streambuf buf;
            while (true) {
                try {
                    boost::asio::read_until(serial_, buf, "\n");
                    std::string data(boost::asio::buffer_cast<const char*>(buf.data()), buf.size());
                    buf.consume(buf.size());  // Clear the buffer
                    callback(data);  // Call the callback with the read data
                } catch (const boost::system::system_error& e) {
                    // Log error and exit the thread if stop has been called or serial port is closed
                    std::cerr << "Read error: " << e.what() << std::endl;
                    break;
                }
            }
        });
    }

    void BoostSerialPort::stop() {
        if (!io_context_.stopped()) {
            io_context_.stop();  // Stop the io_context to cancel any asynchronous operations
        }

        if (serial_.is_open()) {
            serial_.close();  // Close the serial port
        }

        if (read_thread_.joinable()) {
            read_thread_.join();  // Ensure the read thread is finished
        }
    }


    BoostSerialPort::~BoostSerialPort() {
        stop();
    }
} // namespace platform::devices