#include "platform/io/BoostSerialDevice.hpp"

#include "platform/io/Status.hpp"

#include <boost/asio.hpp>

#include <cctype>
#include <iostream>
#include <mutex>
#include <optional>
#include <string>
#include <tuple>
#include <unordered_map>

namespace platform::io {

BoostSerialDevice::BoostSerialDevice(std::string port, uint32_t baudRate, bool isRequestEchoed)
    : port_(std::move(port))
    , baudRate_(baudRate)
    , serialPort_(ioContext_)
    , isRequestEchoed_{isRequestEchoed}
{}

Status BoostSerialDevice::open()
{
    if (!serialPort_.is_open()) {
        serialPort_.open(port_);
        serialPort_.set_option(boost::asio::serial_port_base::baud_rate(baudRate_));
        serialPort_.set_option(boost::asio::serial_port::character_size(8));

        serialPort_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serialPort_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serialPort_.set_option(
            boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

        // Manually controlling the RTS and DTR signals
        serialPort_.set_option(
            boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        serialPort_.send_break(); // Equivalent to toggling DTR in some contexts

        isOpened_ = true;
        return {STATUS::SUCCESS};
    }
    return {STATUS::ERROR, ERROR::ERROR_OPENING_DEVICE};
}

Status BoostSerialDevice::close()
{
    if (serialPort_.is_open()) {
        serialPort_.close();
        isOpened_ = false;
        return {STATUS::SUCCESS};
    }
    return {STATUS::SUCCESS}; // Port was not open
}

bool BoostSerialDevice::isValidRequest(const std::string& request, const std::string& response)
{
    std::unordered_map<char, size_t> charCount;

    // Lambda to increment character counts in the map
    auto countChars = [&charCount](const std::string& str) {
        for (char c : str) {
            if (std::iscntrl(c) == 0) {
                charCount[c]++;
            }
        }
    };

    countChars(request);

    // Using the same map but decrementing to check if counts match
    for (char c : response) {
        if (std::iscntrl(c) == 0) {
            if (charCount[c] == 0) {
                return false; // More chars in response than in request
            }
            charCount[c]--;
        }
    }

    // Check if all counts are zero
    for (const auto& [c, count] : charCount) {
        if (count != 0) {
            return false;
        }
    }
    return true;
}

Status BoostSerialDevice::innerWrite(const std::string& request)
{
    boost::asio::write(serialPort_, boost::asio::buffer(request.c_str(), request.size()));

    // some devices echo the request as a way
    // to indicate successful receipt of the request
    if (isRequestEchoed_) {
        // Read response until newline
        boost::asio::read_until(serialPort_, response_, '\n');

        // the device will echo
        std::istream is(&response_);
        std::string echo;
        std::getline(is, echo);

        // Check if echoed message matches sent message
        if (!isValidRequest(request, echo)) {
            return {STATUS::ERROR, ERROR::COMMUNICATION_ERROR};
        }
    }
    return {STATUS::SUCCESS};
}

Status BoostSerialDevice::write(const std::string& request)
{
    std::lock_guard<std::mutex> lock(ioMutex_);
    if (!isOpened_) {
        return {STATUS::ERROR, ERROR::SENSOR_FAILURE};
    }
    return innerWrite(request);
}

std::tuple<Status, std::optional<std::string>> BoostSerialDevice::read(const std::string& request)
{
    std::lock_guard<std::mutex> lock(ioMutex_);
    if (!isOpened_) {
        return {Status(STATUS::ERROR, ERROR::SENSOR_FAILURE), {}};
    }

    Status status = innerWrite(request);
    if (!status.isSuccess()) {
        return {status, {}};
    }

    std::string received;
    boost::asio::read_until(serialPort_, response_, '\n');
    std::istream is(&response_);
    std::getline(is, received);

    size_t pos = received.find('\n');
    if (pos != std::string::npos) {
        received.erase(pos);
    }
    return {Status(STATUS::SUCCESS), std::make_optional(received)};
}

Status BoostSerialDevice::initialize()
{
    return {};
}

} // namespace platform::io
