#pragma once

#include <boost/asio.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>

#include <chrono>
#include <exception>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace platform::vehicle::waverover {

namespace beast = boost::beast;
namespace http = beast::http;
namespace net = boost::asio;
using tcp = net::ip::tcp;

class WaveRoverNetworkDeviceManager
{
public:
    WaveRoverNetworkDeviceManager()
        : ioc_()
        , socket_(ioc_)
        , delay_(1)
        , max_delay_(300)
    {
        connect(); // Initial connection attempt
    }

    bool connect()
    {
        if (socket_.is_open()) {
            return true; // Already connected
        }

        tcp::resolver resolver{ioc_};
        boost::system::error_code ec;

        boost::asio::ip::tcp::resolver::results_type results = resolver.resolve(server_, port_, ec);
        if (ec) {
            std::cerr << "Resolve error: " << ec.message() << std::endl;
            return false;
        }

        while (true) {
            net::connect(socket_, results, ec);
            if (!ec) {
                delay_ = 1; // Reset delay after successful connection
                return true;
            } else if (ec.value() == boost::asio::error::connection_refused) {
                std::cerr << "Connection refused, retrying..." << std::endl;
                //std::this_thread::sleep_for(std::chrono::seconds(delay_));
                delay_ = std::min(delay_ * 2, max_delay_); // Exponential backoff
            } else {
                std::cerr << "Connection error: " << ec.message() << std::endl;
                return false;
            }
        }
    }

    void write(const std::string& command) {
        std::lock_guard<std::mutex> lock(socketMutex_);

        close();
        connect();

        if (!socket_.is_open() && !connect()) {
            return;
        }

        sendRequest(command, false);
    }

    std::optional<std::string> read() {
        std::lock_guard<std::mutex> lock(socketMutex_);

        close();
        connect();

        if (!socket_.is_open() && !connect()) {
            return std::nullopt;
        }

        auto result = sendRequest(target_, false);
        if (std::holds_alternative<bool>(result) && !std::get<bool>(result)) {
            std::cerr << "holds_alternative returned false" << std::endl;
            return std::nullopt;  // Failed to send the request
        }

        close();
        connect();
        result = sendRequest("/jsfb", true);
        if (std::holds_alternative<std::string>(result)) {
            return std::get<std::string>(result);
        }

        return std::nullopt;
    }

private:
    std::variant<bool, std::string> sendRequest(const std::string& path, bool expectResponse) {
        if (!socket_.is_open() && !connect()) {
            return false;
        }

        try {
            http::request<http::string_body> req{http::verb::get, path, 11};
            req.set(http::field::host, server_);
            req.set(http::field::user_agent, BOOST_BEAST_VERSION_STRING);

            http::write(socket_, req);

            if (expectResponse) {
                beast::flat_buffer buffer;
                http::response<http::dynamic_body> res;
                try {
                    http::read(socket_, buffer, res);
                } catch (const beast::system_error& e) {
                    // If end of stream error, treat as connection closed by server.
                    if (e.code() == beast::http::error::end_of_stream) {
                        close();  // Close the socket for cleanup
                        // Decide based on your application's needs whether to reconnect,
                        // return success (if partial read is okay), or return failure.
                        if (!connect()) {
                            return false;  // Reconnection failed
                        }
                        // If reconnection is successful, you might want to retry the request
                        // or proceed based on the application logic.
                    } else {
                        throw;  // Re-throw the exception for other errors
                    }
                }
                return beast::buffers_to_string(res.body().data());
            }

            return true;
        } catch (const beast::system_error& e) {
            std::cerr << "Send request error: " << e.what() << std::endl;
            close();  // Ensure the socket is closed on error
            return false;
        }
    }


    void close()
    {
        if (socket_.is_open()) {
            boost::system::error_code ec;
            socket_.shutdown(tcp::socket::shutdown_both, ec);
            socket_.close(ec);
        }
    }

    net::io_context ioc_;
    tcp::socket socket_;
    std::mutex socketMutex_;
    std::string server_ = "192.168.4.1";
    std::string port_ = "80";
    std::string target_ = "/js?json={\"T\":71}";

    unsigned int delay_;
    const unsigned int max_delay_;
};

} // namespace platform::vehicle::waverover
