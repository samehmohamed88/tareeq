#include <iostream>
#include <boost/asio.hpp>
#include <thread>
#include <chrono>

int main() {
    using namespace boost::asio;
    io_service io;
    serial_port port(io);

    try {
        port.open("/dev/ttyUSB0");
        port.set_option(serial_port_base::baud_rate(115200));
        port.set_option(serial_port_base::character_size(8));
        port.set_option(serial_port_base::parity(serial_port_base::parity::none));
        port.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
        port.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));

        // Manually controlling the RTS and DTR signals
        port.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
        port.send_break(); // Equivalent to toggling DTR in some contexts
//        port.clear(); // Clear the serial port's buffer and status

        // Write JSON command followed by newline
        std::string message = "{\"T\":126}\n";
        write(port, buffer(message.c_str(), message.size()));

        // Read response until newline (simplified version)
        boost::asio::streambuf response;
        boost::asio::read_until(port, response, '\n');

        std::istream is(&response);
        std::string received;
        std::getline(is, received);
        std::cout << "Received: -- " << received << std::endl;

        boost::asio::read_until(port, response, '\n');
        std::getline(is, received);
        std::cout << "Received: X " << received << std::endl;

//        std::string resp;
//        auto size = boost::asio::read(port, boost::asio::buffer(resp, 10*1024));
//        std::cout << " >>>>>>> " << resp << std::endl;
//        while (size > 0) {
//            size = boost::asio::read(port, boost::asio::buffer(resp, 10*1024));
//            std::cout << " >>>>>>> " << resp << std::endl;
//        }

    } catch (boost::system::system_error& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    port.close();
    return 0;
}
