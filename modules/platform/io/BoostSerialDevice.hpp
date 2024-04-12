#pragma once

#include "platform/io/IOInterface.hpp"
#include "platform/io/Status.hpp"

#include <boost/asio.hpp>

#include <optional>
#include <string>
#include <tuple>

namespace platform::io {

/// Represents a serial device using Boost.Asio to handle serial communications.
/// This class encapsulates all the functionalities required to open, close, read from,
/// and write to a serial port. It uses Boost.Asio's asynchronous I/O operations to manage
/// serial port data transmission and reception.
class BoostSerialDevice : public IOInterface<std::string>
{
public:
    /// Constructs a serial device interface.
    /// @param port The name of the serial port (e.g., COM1 or /dev/ttyUSB0).
    /// @param baudRate The baud rate for the serial connection.
    /// @param isRequestEchoed Flag to determine if the device echoes the sent requests.
    BoostSerialDevice(std::string port, uint32_t baudRate, bool isRequestEchoed = true);

    /// Initializes the serial port settings.
    /// This method sets up the serial port with the specified configurations and prepares it for use.
    /// @return Status indicating success or failure of the initialization.
    Status initialize() override;

    /// Opens the serial port for communication.
    /// It ensures that the port is configured and ready for sending and receiving data.
    /// @return Status indicating success or failure of opening the port.
    Status open() override;

    /// Closes the serial port.
    /// Ensures that the port is properly closed and any associated resources are freed.
    /// @return Status indicating success or failure of closing the port.
    Status close() override;

    /// Writes data to the serial port.
    /// Sends a string to the serial port and optionally checks for an echo if required.
    /// @param request The string to send to the serial port.
    /// @return Status indicating success or failure of the write operation.
    Status write(const std::string& request) override;

    /// Reads data from the serial port.
    /// Sends a request and reads the response from the serial port.
    /// @param request The request to send before reading the response.
    /// @return A tuple containing a Status object and an optional string with the received data.
    std::tuple<Status, std::optional<std::string>> read(const std::string& request) override;

    /// Checks if the serial port is open.
    /// @return True if the serial port is open, false otherwise.
    const bool isOpened() const override { return isOpened_; };

    /// Checks if the serial port is initialized.
    /// @return True if the serial port is initialized, false otherwise.
    const bool isInitialized() const override { return isInitialized_; };

    const std::string& getDeviceName() const {return port_;};

private:
    /// Handles the internal write operations.
    /// This private method is used to encapsulate the low-level write functionalities.
    /// @param message The message to be written to the serial port.
    /// @return Status indicating success or failure of the internal write operation.
    Status innerWrite(const std::string& message);

    /// Validates the response against the request.
    /// Checks if the response received from the device matches the sent request, based on the configured behavior.
    /// @param request The original request sent to the device.
    /// @param response The response received from the device.
    /// @return True if the response is valid, false otherwise.
    bool isValidRequest(const std::string& request, const std::string& response);

    const std::string port_;              ///< Serial port identifier.
    uint32_t baudRate_;                   ///< Baud rate for serial communication.
    boost::asio::io_context ioContext_;   ///< Boost.Asio IO context for managing I/O services.
    boost::asio::serial_port serialPort_; ///< Boost.Asio serial port object.
    bool isRequestEchoed_;                ///< Flag indicating whether the device echoes back the sent requests.
    boost::asio::streambuf response_;      ///< Buffer for storing responses from the serial port.
};

} // namespace platform::io
