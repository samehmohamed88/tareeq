#pragma once

#include "platform/devices/ISerialPort.hpp"

#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>
#include <boost/asio.hpp>
#include <boost/beast.hpp>

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <iostream>

namespace platform::devices {

namespace beast = boost::beast;
namespace http = beast::http;
namespace net = boost::asio;
using tcp = net::ip::tcp;

template<typename AsioOperations, typename ILogger>
class BoostNetworkDevice : public ISerialPort
{
public:
    BoostNetworkDevice(std::shared_ptr<AsioOperations> asioOperations,
                       std::shared_ptr<ILogger> logger,
                       std::string server = "192.168.4.1",
                       std::string port = "80",
                       std::string localAddress = "192.168.4.3");

    ~BoostNetworkDevice() override;

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
    boost::asio::ip::tcp::resolver resolver_;
    boost::asio::ip::tcp::socket socket_;
    std::shared_ptr<ILogger> logger_;

    std::shared_ptr<AsioOperations> asioOperations_;

    const std::string server_;
    const std::string port_;
    const std::string localAddress_;
};

template<typename AsioOperations, typename ILogger>
BoostNetworkDevice<AsioOperations, ILogger>::BoostNetworkDevice(std::shared_ptr<AsioOperations> asioOperations,
                                                                std::shared_ptr<ILogger> logger,
                                                                std::string server,
                                                                std::string port,
                                                                std::string localAddress)
    : resolver_(io_context_)
    , socket_(io_context_)
    , logger_{logger}
    , asioOperations_{asioOperations}
    , server_{std::move(server)}
    , port_{std::move(port)}
    , localAddress_{std::move(localAddress)}

{}

template<typename AsioOperations, typename ILogger>
void BoostNetworkDevice<AsioOperations, ILogger>::initialize() {
    if (isInitialized && socket_.is_open()) {
        logger_->logInfo("Network device already initialized.");
        return;
    }

    try {
        auto const results = resolver_.resolve(server_, port_);
        boost::asio::connect(socket_, results.begin(), results.end());
        isInitialized = true;
        logger_->logInfo("Connection established with " + server_ + ":" + port_);
    } catch (const std::exception& e) {
        logger_->logError("Connection failed: " + std::string(e.what()));
        isInitialized = false;
        if (socket_.is_open()) {
            socket_.close();  // Ensure the socket is closed on failure
        }
    }
}


template<typename AsioOperations, typename ILogger>
void BoostNetworkDevice<AsioOperations, ILogger>::write(const std::string& command) {
    logger_->logInfo("BoostNetworkDevice::write data " + command);

    if (!isInitialized) {
        logger_->logError("Network device not initialized. Attempting to initialize...");
        initialize();
        if (!isInitialized) {
            logger_->logError("Initialization failed.");
            return;
        }
    }

    try {
        http::request<http::string_body> req{http::verb::get, "/js?json=" + command, 11};
        req.set(http::field::host, server_);
        req.set(http::field::user_agent, BOOST_BEAST_VERSION_STRING);

        http::write(socket_, req);

        // Since we don't need to process the response, we can ignore it or log it for debugging
        beast::flat_buffer buffer;
        http::response<http::dynamic_body> res;
        http::read(socket_, buffer, res);  // Consider removing if not needed

        logger_->logInfo("HTTP request sent successfully.");

        // Check if the connection should be closed
        if (!res.keep_alive()) {
            logger_->logInfo("Connection closed by server.");
            socket_.shutdown(tcp::socket::shutdown_both);
            socket_.close();
            isInitialized = false;
        }
    } catch (const beast::system_error& e) {
        logger_->logError("Error sending request: " + std::string(e.what()));
        socket_.close();
        isInitialized = false;
    }
}



template<typename AsioOperations, typename ILogger>
void BoostNetworkDevice<AsioOperations, ILogger>::read(const ReadCallback& callback)
{}

template<typename AsioOperations, typename ILogger>
void BoostNetworkDevice<AsioOperations, ILogger>::stop()
{
    if (socket_.is_open()) {
        boost::system::error_code ec;
        socket_.shutdown(tcp::socket::shutdown_both, ec);
        socket_.close(ec);
        if (ec) {
            logger_->logError("Error closing socket: " + ec.message());
        }
        isInitialized = false;
    }

    if (!io_context_.stopped()) {
        io_context_.stop();
    }

    if (read_thread_.joinable()) {
        read_thread_.join();
    }
}


template<typename AsioOperations, typename ILogger>
BoostNetworkDevice<AsioOperations, ILogger>::~BoostNetworkDevice()
{
    stop();
}

} // namespace platform::devices
