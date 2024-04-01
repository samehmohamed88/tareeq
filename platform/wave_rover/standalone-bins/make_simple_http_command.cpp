#include <boost/beast.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <iostream>
#include <string>

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
        std::cerr << "Error: " << e.what() << std::endl;
    }
}

int main() {
    std::string host = "192.168.4.1"; // or whatever your server is
    std::string port = "80";
    std::string target = R"(/js?json={"T":1,"L":164,"R":164})"; // replace with your actual URL path
//    std::string target = R"(/js?json={"T":1,"L":164,"R":164})"; // replace with your actual URL path

    make_http_request(host, port, target);

    return 0;
}
