#include "platform/vehicle/wave_rover/WaveRoverUtils.hpp"

#include <boost/asio.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>

#include <chrono>
#include <iostream>
#include <string>
#include <thread>

namespace beast = boost::beast;
namespace http = beast::http;
namespace net = boost::asio;
using tcp = net::ip::tcp;
using namespace platform::vehicle::waverover::utils;

int main()
{

    std::string host = "192.168.4.1"; // or whatever your server is
    std::string port = "80";
    //    std::string target = R"(/js?json={"T":71})"; // replace with your actual URL path
    std::string target = R"(/js?json={"T":1,"L":88,"R":88})";

    std::vector<double> gyro_Z;

    for (int i = 0; i < 200; i++) {

        try {

            net::io_context ioc;

            tcp::resolver resolver{ioc};
            tcp::socket socket{ioc};

            std::cout << "created socket -- " << std::endl;

            const boost::asio::ip::tcp::resolver::results_type results = resolver.resolve(host, port);

            std::cout << "connecting socket" << std::endl;

            boost::asio::connect(socket, results.begin(), results.end());

            std::cout << "making request" << std::endl;
            http::request<http::string_body> cmd_req{http::verb::get, target, 11};

            std::cout << "setting host" << std::endl;
            cmd_req.set(http::field::host, host);

            std::cout << "setting user agent" << std::endl;
            cmd_req.set(http::field::user_agent, BOOST_BEAST_VERSION_STRING);

            std::cout << "http write" << std::endl;
            http::write(socket, cmd_req);

            std::cout << "closing socket" << std::endl;
            socket.close();

            //
            //            const boost::asio::ip::tcp::resolver::results_type results = resolver.resolve(host, port);
            //
            //            net::connect(socket, results.begin(), results.end());
            //
            //            http::request<http::string_body> req{http::verb::get, target, 11};
            //            req.set(http::field::host, host);
            //            req.set(http::field::user_agent, BOOST_BEAST_VERSION_STRING);
            //
            //            http::write(socket, req);
            //
            //            socket.close();

            //            net::connect(socket, results.begin(), results.end());
            //            http::request<http::empty_body> data_req{http::verb::get, "/jsfb", 11};
            //            data_req.set(http::field::host, host);
            //
            //            http::write(socket, data_req);
            //
            //            // Read the response
            //            beast::flat_buffer buffer;
            //            http::response<http::dynamic_body> res;
            //
            //            http::read(socket, buffer, res);
            //
            //            // Convert and return the response body as a string
            //            std::string responseData = beast::buffers_to_string(res.body().data());
            //                    std::cout << "Response received: " << responseData << std::endl;
            //
            //            auto imuData = jsonToIMUData(responseData);
            //            gyro_Z.push_back(imuData.gyro_Z);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        } catch (...) {
            std::cout << "ERROR occurred" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    for (auto d : gyro_Z) {
        std::cout << std::to_string(d) << " ";
    }
    std::cout << std::endl;

    return 0;
}
