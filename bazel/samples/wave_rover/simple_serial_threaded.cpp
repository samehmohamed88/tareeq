#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>

using namespace boost::asio;
using namespace std;

io_service io;
serial_port* sp = nullptr;
bool running = true;

void read_serial() {
    try {
        while (running) {
//            boost::asio::streambuf buf;
            std::string response;
            std::size_t read_size = boost::asio::read(*sp, boost::asio::buffer(response, 100));
            std::cout << " >>>>>>>> " << read_size << std::endl;
            if (read_size == 0u) {
                std::cout << " response " << response << std::endl;
            }
//            if (read_size > 0) {
//                std::istream is(&buf);
//                std::string line, full_message;
//                while (std::getline(is, line)) {
//                    if (!line.empty()) {
//                        full_message += line + "\n";  // Append each line to the full message
//                    }
//                }
//                if (!full_message.empty()) {
//                    std::cout << "Received: " << full_message;  // Print all at once
//                }
//            }
        }
    } catch (std::exception& e) {
        std::cerr << "Read failed: " << e.what() << std::endl;
        running = false;  // Optionally stop the loop on a failure
    }
}


int main(int argc, char* argv[]) {
    try {
        std::string serial_port_name("/dev/ttyUSB0");
        // Set up the serial port
        sp = new serial_port(io, serial_port_name);
        sp->set_option(serial_port_base::baud_rate(115200));
        sp->set_option(serial_port_base::character_size(8));
        sp->set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
        sp->set_option(serial_port_base::parity(serial_port_base::parity::none));
        sp->set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));

        // Start the reading thread
        boost::thread reader_thread(read_serial);

        // Send data
        string command = "{\"T\":126}\n";
        write(*sp, buffer(command + '\n'));

        // Use Ctrl+C to stop the program
        reader_thread.join();
    }
    catch (std::exception& e) {
        cerr << "Exception: " << e.what() << "\n";
    }

    if (sp) {
        sp->close();
        delete sp;
    }

    return 0;
}
