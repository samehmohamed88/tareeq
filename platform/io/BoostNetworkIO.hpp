#pragma once

#include "platform/io/IOInterface.hpp"

#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>

#include <array>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>

namespace platform::io {

namespace beast = boost::beast;
namespace http = beast::http;
namespace net = boost::asio;
using tcp = net::ip::tcp;

template<typename AsioOperations, typename ILogger>
class BoostNetworkIO : public IOInterface
{
public:
    BoostNetworkIO(std::shared_ptr<AsioOperations> asioOperations,
                   std::shared_ptr<ILogger> logger,
                   std::string server = "192.168.4.1",
                   std::string port = "80",
                   std::string localAddress = "192.168.4.3");

    ~BoostNetworkIO() override;

    void write(const std::string& request) override;

    std::optional<std::string> read(const std::string& request) override;
    void initialize();
    bool reconnect();

private:
    void stop();
    void initializeBaseRequest();
    bool prepareRequest(const std::string& request);

    bool isInitialized = false;
    std::thread read_thread_;
    std::mutex read_mutex_;  // Mutex to synchronize access to the read method
    std::mutex write_mutex_; // Mutex to synchronize access to the write method

    http::request<http::string_body> base_request_;

    std::shared_ptr<ILogger> logger_;

    std::shared_ptr<AsioOperations> asioOperations_;

    const std::string server_;
    const std::string port_;
    const std::string localAddress_;
    //    const boost::asio::ip::tcp::resolver::results_type results_;
};

template<typename AsioOperations, typename ILogger>
BoostNetworkIO<AsioOperations, ILogger>::BoostNetworkIO(std::shared_ptr<AsioOperations> asioOperations,
                                                        std::shared_ptr<ILogger> logger,
                                                        std::string server,
                                                        std::string port,
                                                        std::string localAddress)
    //    : resolver_(io_context_)
    //    , socket_(io_context_)
    : logger_{logger}
    , asioOperations_{asioOperations}
    , server_{std::move(server)}
    , port_{std::move(port)}
    , localAddress_{std::move(localAddress)}
//    , results_{}

{}

// Method to initialize the base request
template<typename AsioOperations, typename ILogger>
void BoostNetworkIO<AsioOperations, ILogger>::initializeBaseRequest()
{
    base_request_.method(http::verb::get);
    base_request_.version(11); // HTTP/1.1
    base_request_.set(http::field::host, server_);
    base_request_.set(http::field::user_agent, BOOST_BEAST_VERSION_STRING);
    // Prepare the payload to set the Content-Length and other necessary headers
    base_request_.prepare_payload();
}

// Method to initialize the base request
template<typename AsioOperations, typename ILogger>
bool BoostNetworkIO<AsioOperations, ILogger>::prepareRequest(const std::string& request)
{
    // logger_->logInfo("BoostNetworkIO::write prepareRequest " + request);

    if (!isInitialized) {
        logger_->logError("Network device not initialized. Attempting to initialize...");
        initialize();
        if (!isInitialized) {
            logger_->logError("Initialization failed.");
            return false;
        }
    }
    return true;
}

template<typename AsioOperations, typename ILogger>
void BoostNetworkIO<AsioOperations, ILogger>::initialize()
{
    //    if (isInitialized && socket_.is_open()) {
    //        // logger_->logInfo("Network device already initialized.");
    //        return;
    //    }
    //
    //    try {
    ////        auto const results = resolver_.resolve(server_, port_);
    //
    //
    ////        initializeBaseRequest();
    //
    //        isInitialized = true;
    //        // logger_->logInfo("Connection established with " + server_ + ":" + port_);
    //    } catch (const std::exception& e) {
    //        logger_->logError("Connection failed: " + std::string(e.what()));
    //        isInitialized = false;
    //        if (socket_.is_open()) {
    //            socket_.close();  // Ensure the socket is closed on failure
    //        }
    //    }
}

template<typename AsioOperations, typename ILogger>
void BoostNetworkIO<AsioOperations, ILogger>::write(const std::string& request)
{
    //    boost::asio::io_context io_context_;
    //    boost::asio::ip::tcp::resolver resolver_{io_context_};
    //    boost::asio::ip::tcp::socket socket_{io_context_};
    //
    //    try {
    //
    //        const boost::asio::ip::tcp::resolver::results_type results_ = resolver_.resolve(server_, port_);
    //
    //        boost::asio::connect(socket_, results_.begin(), results_.end());
    //        http::request<http::string_body> cmd_req{http::verb::get, request, 11};
    //        cmd_req.set(http::field::host, server_);
    //        cmd_req.set(http::field::user_agent, BOOST_BEAST_VERSION_STRING);
    //        http::write(socket_, cmd_req);
    //        socket_.close();
    //
    //    } catch (const beast::system_error& e) {
    //        logger_->logError("Errors sending request: " + std::string(e.what()));
    //        socket_.close();
    //        isInitialized = false;
    //    } catch (...) {
    //        socket_.close();
    //    }

    try {
        net::io_context ioc;

        tcp::resolver resolver{ioc};
        tcp::socket socket{ioc};

        const boost::asio::ip::tcp::resolver::results_type results = resolver.resolve(server_, port_);

        net::connect(socket, results.begin(), results.end());

        http::request<http::string_body> req{http::verb::get, request, 11};
        req.set(http::field::host, server_);
        req.set(http::field::user_agent, BOOST_BEAST_VERSION_STRING);

        http::write(socket, req);

        socket.close();

    } catch (...) {
        std::cout << "ERROR occurred" << std::endl;
        //        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

template<typename AsioOperations, typename ILogger>
bool BoostNetworkIO<AsioOperations, ILogger>::reconnect()
{
    //    try {
    //
    //        // Close the socket after the command request
    ////        socket_.shutdown(tcp::socket::shutdown_both);
    //        socket_.close();
    //
    //        // Resolve the server address and port
    ////        auto const results = resolver_.resolve(server_, port_);
    //
    //        // Create a new socket and connect it to the server
    //        socket_ = tcp::socket(io_context_);
    //        boost::asio::connect(socket_, results_.begin(), results_.end());
    //
    //        // Update the initialization status
    //        isInitialized = true;
    //        // logger_->logInfo("Reconnected to " + server_ + ":" + port_);
    //
    //        return true;
    //    } catch (const std::exception& e) {
    //        logger_->logError("Reconnect failed: " + std::string(e.what()));
    //        isInitialized = false;
    //        return false;
    //    }
}

template<typename AsioOperations, typename ILogger>
std::optional<std::string> BoostNetworkIO<AsioOperations, ILogger>::read(const std::string& requestold)
{

    //    boost::asio::io_context io_context_;
    //    boost::asio::ip::tcp::resolver resolver_{io_context_};
    //    boost::asio::ip::tcp::socket socket_{io_context_};
    //
    //    try {
    //
    //        const boost::asio::ip::tcp::resolver::results_type results_ = resolver_.resolve(server_, port_);
    //        boost::asio::connect(socket_, results_.begin(), results_.end());
    //        http::request<http::empty_body> data_req{http::verb::get, request, 11};
    //        data_req.set(http::field::host, server_);
    //        http::write(socket_, data_req);
    //
    //        // Read the response
    //        beast::flat_buffer buffer;
    //        http::response<http::dynamic_body> res;
    //        http::read(socket_, buffer, res);
    //
    //        // Convert and return the response body as a string
    //        std::string responseData = beast::buffers_to_string(res.body().data());
    //
    //        socket_.close();
    //
    //        return responseData;
    //
    //    } catch (const beast::system_error& e) {
    //        logger_->logError("Errors in read operation: " + std::string(e.what()));
    //        isInitialized = false;
    //        socket_.close();
    //    } catch (...) {
    //        socket_.close();
    //    }

    try {
        std::string target = R"(/js?json={"T":71})"; // replace with your actual URL path
        net::io_context ioc;

        tcp::resolver resolver{ioc};
        tcp::socket socket{ioc};

        const boost::asio::ip::tcp::resolver::results_type results = resolver.resolve(server_, port_);

        net::connect(socket, results.begin(), results.end());

        http::request<http::string_body> req{http::verb::get, target, 11};
        req.set(http::field::host, server_);
        req.set(http::field::user_agent, BOOST_BEAST_VERSION_STRING);

        http::write(socket, req);

        socket.close();

        net::connect(socket, results.begin(), results.end());
        http::request<http::empty_body> data_req{http::verb::get, "/jsfb", 11};
        data_req.set(http::field::host, server_);

        http::write(socket, data_req);

        // Read the response
        beast::flat_buffer buffer;
        http::response<http::dynamic_body> res;

        http::read(socket, buffer, res);

        // Convert and return the response body as a string
        std::string responseData = beast::buffers_to_string(res.body().data());
        //        std::cout << "Response received: " << responseData << std::endl;

        //        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } catch (...) {
        std::cout << "ERROR occurred" << std::endl;
        //        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return std::nullopt;
}

template<typename AsioOperations, typename ILogger>
void BoostNetworkIO<AsioOperations, ILogger>::stop()
{
    //    if (socket_.is_open()) {
    //        boost::system::error_code ec;
    //        socket_.shutdown(tcp::socket::shutdown_both, ec);
    //        socket_.close(ec);
    //        if (ec) {
    //            logger_->logError("Errors closing socket: " + ec.message());
    //        }
    //        isInitialized = false;
    //    }
    //
    //    if (!io_context_.stopped()) {
    //        io_context_.stop();
    //    }
    //
    //    if (read_thread_.joinable()) {
    //        read_thread_.join();
    //    }
}

template<typename AsioOperations, typename ILogger>
BoostNetworkIO<AsioOperations, ILogger>::~BoostNetworkIO()
{
    stop();
}

} // namespace platform::io
