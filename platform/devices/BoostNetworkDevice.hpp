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

namespace beast = boost::beast; // from <boost/beast.hpp>
namespace http = beast::http; // from <boost/beast/http.hpp>
namespace net = boost::asio; // from <boost/asio.hpp>
using tcp = net::ip::tcp; // from <boost/asio/ip/tcp.hpp>

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
void BoostNetworkDevice<AsioOperations, ILogger>::initialize()
{
    if (!isInitialized) {
        logger_->logInfo("Initializing the BoostHttpClient with server " + server_ + " and port " + port_);

        if (!localAddress_.empty()) {
            // Bind the socket to the local address (interface) before connecting
            boost::asio::ip::tcp::endpoint local_endpoint(boost::asio::ip::make_address(localAddress_), 0);
            socket_.open(boost::asio::ip::tcp::v4());
            socket_.bind(local_endpoint);
        }

        boost::asio::ip::tcp::resolver::query query(server_, port_);
        auto endpoint_iterator = resolver_.resolve(query);
        boost::asio::connect(socket_, endpoint_iterator);

        isInitialized = true;
    }
}

template<typename AsioOperations, typename ILogger>
void BoostNetworkDevice<AsioOperations, ILogger>::write(const std::string& command)
{
    logger_->logInfo("BoostNetworkDevice::write data " + command);
    std::lock_guard<std::mutex> lock(write_mutex_);
//    try {
//        logger_->logInfo("Sending HTTP request: " + request);
////        boost::asio::write(socket_, boost::asio::buffer(request));
//        asioOperations_->template write<boost::asio::ip::tcp::socket>(socket_, boost::asio::buffer(request));
//    } catch (const boost::system::system_error& e) {
//        logger_->logError("BoostNetworkDevice : Error while writing: " + request);
//        logger_->logError("BoostNetworkDevice : Error while writing: " + std::string(e.what()));
//        throw e;
//    }

    try {
        std::string target = "/js?json=" + command;
        std::string host = server_ + ":" + port_;

        // Set up an HTTP GET request message
        http::request<http::string_body> req{http::verb::get, target, 11};
        req.set(http::field::host, host);
        req.set(http::field::user_agent, BOOST_BEAST_VERSION_STRING);

        // Look up the domain name
        auto const results = resolver_.resolve(server_, port_);

        // Make the connection on the IP address we get from a lookup
        net::connect(socket_, results.begin(), results.end());

        // Send the HTTP request to the remote host
        http::write(socket_, req);

        // This buffer is used for reading and must be persisted
        beast::flat_buffer buffer;

        // Declare a container to hold the response
        http::response<http::dynamic_body> res;

        // Receive the HTTP response
        http::read(socket_, buffer, res);

        // Write the message to standard out
        std::cout << res << std::endl;

        // Gracefully close the socket
        beast::error_code ec;
        socket_.shutdown(tcp::socket::shutdown_both, ec);

        // not_connected happens sometimes so don't bother reporting it.
        if (ec && ec != beast::errc::not_connected)
            throw beast::system_error{ec};

        // If we get here then the connection is closed gracefully
    } catch (std::exception const& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

}

template<typename AsioOperations, typename ILogger>
void BoostNetworkDevice<AsioOperations, ILogger>::read(const ReadCallback& callback)
{}

template<typename AsioOperations, typename ILogger>
void BoostNetworkDevice<AsioOperations, ILogger>::stop()
{
    if (!io_context_.stopped()) {
        io_context_.stop(); // Stop the io_context to cancel any asynchronous operations
    }

    // Since there's no direct is_open() check for a socket, we can use the lowest layer's is_open()
    // if you're using a plain TCP socket, or connected() if you have a higher-level protocol or session management
    if (socket_.is_open()) {
        socket_.close(); // Close the socket
    }

    if (read_thread_.joinable()) {
        read_thread_.join(); // Ensure the read thread is finished
    }
}

template<typename AsioOperations, typename ILogger>
BoostNetworkDevice<AsioOperations, ILogger>::~BoostNetworkDevice()
{
    stop();
}

} // namespace platform::devices
