#include <boost/beast.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>
#include <boost/asio.hpp>
#include <boost/beast.hpp>

#include <iostream>
#include <string>
#include <thread>
#include <chrono>



namespace beast = boost::beast;
namespace http = beast::http;
namespace net = boost::asio;
using tcp = net::ip::tcp;

void make_http_request(const std::string& host, const std::string& port, const std::string& target) {
    try {
        net::io_context ioc;

        tcp::resolver resolver{ioc};
        tcp::socket socket{ioc};

        auto const results = resolver.resolve(host, port);
        net::connect(socket, results.begin(), results.end());

        http::request<http::string_body> req{http::verb::get, target, 11};
        req.set(http::field::host, host);
        req.set(http::field::user_agent, BOOST_BEAST_VERSION_STRING);

        http::write(socket, req);

        beast::flat_buffer buffer;
        http::response<http::dynamic_body> res;

        http::read(socket, buffer, res);

        std::cout << res << std::endl;

        socket.shutdown(tcp::socket::shutdown_both);

    } catch (std::exception const& e) {
        std::cerr << "Errors: " << e.what() << std::endl;
    }
}

int main() {


    for (int i = 0 ; i < 100; i ++) {
        std::string host = "192.168.4.1"; // or whatever your server is
        std::string port = "80";
        std::string target = R"(/js?json={"T":71})"; // replace with your actual URL path

        net::io_context ioc;

        tcp::resolver resolver{ioc};
        tcp::socket socket{ioc};


        net::connect(socket, results.begin(), results.end());

        http::request<http::string_body> req{http::verb::get, target, 11};
        req.set(http::field::host, host);
        req.set(http::field::user_agent, BOOST_BEAST_VERSION_STRING);

        http::write(socket, req);

        socket.close();



        net::connect(socket, results.begin(), results.end());
        http::request<http::empty_body> data_req{http::verb::get, "/jsfb", 11};
        data_req.set(http::field::host, host);

        http::write(socket, data_req);

        // Read the response
        beast::flat_buffer buffer;
        http::response<http::dynamic_body> res;

        http::read(socket, buffer, res);

        // Convert and return the response body as a string
        std::string responseData = beast::buffers_to_string(res.body().data());
        std::cout << "Response received: " << responseData << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }



    return 0;
}
