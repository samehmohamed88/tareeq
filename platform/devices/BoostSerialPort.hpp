#pragma once

#include "platform/devices/ISerialPort.hpp"

#include <boost/asio.hpp>

#include <array>
#include <mutex>
#include <string>
#include <thread>
#include <memory>
#include <utility>

namespace platform::devices {

template <typename AsioOperations, typename ILogger>
class BoostSerialPort : public ISerialPort
{
public:
    BoostSerialPort(std::shared_ptr<AsioOperations> asioOperations,
                    std::shared_ptr<ILogger> logger,
                    std::string port,
                    uint32_t baudRate = 115200);

    ~BoostSerialPort() override;

    void write(const std::string& data) override;

    void read(const ReadCallback& callback) override;
    void initialize();

private:
    void stop();


    bool isInitialized = false;
    std::thread read_thread_;
    std::mutex read_mutex_;  // Mutex to synchronize access to the read method
    std::mutex write_mutex_; // Mutex to synchronize access to the write method

    boost::asio::io_context io_context_;
    boost::asio::serial_port serial_;

    std::shared_ptr<ILogger> logger_;
    std::shared_ptr<AsioOperations> asioOperations_;

    const std::string port_;
    uint32_t baudRate_;

};

template <typename AsioOperations, typename ILogger>
BoostSerialPort<AsioOperations, ILogger>::BoostSerialPort(std::shared_ptr<AsioOperations> asioOperations,
                                                          std::shared_ptr<ILogger> logger,
                                                          std::string port,
                                                          uint32_t baudRate)
    : serial_(io_context_)
    , logger_{logger}
    , asioOperations_{asioOperations}
    , port_{std::move(port)}
    , baudRate_{baudRate}
{

}

template <typename AsioOperations, typename ILogger>
void BoostSerialPort<AsioOperations, ILogger>::initialize() {
    if (!isInitialized) {
        logger_->logInfo(">>>>>>>>>> Initializing the BoostSerialPort boost::asio::serial_port object with port " + port_ + " and baud rate " + std::to_string(baudRate_));
        serial_.open(port_);
        serial_.set_option(boost::asio::serial_port_base::baud_rate(baudRate_));
//        serial_.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
//        serial_.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
//        serial_.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
//        serial_.set_option(boost::asio::serial_port::character_size(8));
    }
}

template <typename AsioOperations, typename ILogger>
void BoostSerialPort<AsioOperations, ILogger>::write(const std::string& data)
{
    logger_->logInfo("BoostSerialPort::write data " + data);
    std::lock_guard<std::mutex> lock(write_mutex_);
    try {
        logger_->logInfo("BoostSerialPort::write calling boost asio operations impl");
        asioOperations_->write(serial_, boost::asio::buffer(data));
    } catch (const boost::system::system_error& e) {
        logger_->logError("BoostSerialPort : Error while writing: " + data);
        logger_->logError("BoostSerialPort : Error while writing: " + std::string(e.what()));
        throw e;
    }
}

template <typename AsioOperations, typename ILogger>
void BoostSerialPort<AsioOperations, ILogger>::read(const ReadCallback& callback)
{
//    std::lock_guard<std::mutex> lock(read_mutex_);
//    if (read_thread_.joinable()) {
//        // Wait for any existing read operation to finish before starting a new one
//        read_thread_.join();
//    }
//
//    read_thread_ = std::thread([this, &callback]() {
//        boost::asio::streambuf buf;
//        while (true) {
//            try {
//                asioOperations_->read_until(serial_, buf, "\n");
//                std::string data(boost::asio::buffer_cast<const char*>(buf.data()), buf.size());
//                buf.consume(buf.size()); // Clear the buffer
//                callback(data);          // Call the callback with the read data
//            } catch (const boost::system::system_error& e) {
//                // Log error and exit the thread if stop has been called or serial port is closed
//                logger_->logError("Write error: " + std::string(e.what()));
//                break;
//            }
//        }
//    });
}

template <typename AsioOperations, typename ILogger>
void BoostSerialPort<AsioOperations, ILogger>::stop()
{
    if (!io_context_.stopped()) {
        io_context_.stop(); // Stop the io_context to cancel any asynchronous operations
    }

    if (serial_.is_open()) {
        serial_.close(); // Close the serial port
    }

    if (read_thread_.joinable()) {
        read_thread_.join(); // Ensure the read thread is finished
    }
}

template <typename AsioOperations, typename ILogger>
BoostSerialPort<AsioOperations, ILogger>::~BoostSerialPort()
{
    stop();
}

} // namespace platform::devices
