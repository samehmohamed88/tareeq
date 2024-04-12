#pragma once

#include "platform/io/Status.hpp"

#include <exception>
#include <optional>
#include <string>
#include <tuple>
#include <mutex>

namespace platform::io {
/// @class IOInterface
/// @brief Interface class for input/output operations.
///
/// This interface defines the basic operations required for
/// input and output functionality in a device or file handling class.
template<typename T>
class IOInterface
{
public:
    /// @brief Virtual destructor for the interface.
    virtual ~IOInterface() = default;

    /// @brief Open a connection or file.
    ///
    /// Opens a connection or file and prepares it for use.
    /// This method must be implemented by the derived class to
    /// handle specific open operations.
    ///
    /// @return Status indicating the result of the operation. Success or Error details.
    virtual Status open() = 0;

    /// @brief Close the current connection or file.
    ///
    /// Closes the currently open connection or file. It is important
    /// to ensure that all resources are properly released.
    /// This method must be implemented by the derived class to handle
    /// specific close operations.
    ///
    /// @return Status indicating the result of the operation. Success or Error details.
    virtual Status close() = 0;

    /// @brief Reopen a connection or file.
    ///
    /// Closes the current connection or file if open, and reopens it.
    /// This method can be used to reset the state or recover from errors.
    /// Implementation should ensure that all relevant resources are cleanly
    /// reinitialized.
    ///
    /// @return Status indicating the result of the operation. Success or Error details.
    virtual Status reopen()
    {
        std::lock_guard<std::mutex> lock(ioMutex_);
        try {
            if (isOpened()) {
                close();
            }
            open();
            return initialize();
        } catch (const std::exception& e) {
            return {STATUS::ERROR, ERROR::COMMUNICATION_ERROR};
        }
    };

    /// @brief Initialize the connection or file after opening.
    ///
    /// Performs any necessary initialization steps after opening the connection or file.
    /// This method must be implemented by the derived class to handle specific initialization procedures.
    ///
    /// @return Status indicating the success or failure of the initialization.
    virtual Status initialize() = 0;

    /// @brief Write data or a request to the connection or file.
    ///
    /// Sends a request or writes data to the open connection or file.
    /// This method must be implemented by the derived class to handle specific write operations.
    ///
    /// @param request The data or request to write.
    /// @return Status indicating the result of the write operation.
    virtual Status write(const std::string& request) = 0;

    /// @brief Read data or a response from the connection or file.
    ///
    /// Reads data or a response based on a request from the open connection or file.
    /// This method must be implemented by the derived class to handle specific read operations.
    ///
    /// @param request The request for which the response is desired.
    /// @return A tuple containing the status of the read operation and the optional data received.
    virtual std::tuple<Status, std::optional<T>> read(const std::string& request) = 0;

    /// @brief Check if the connection or file is currently open.
    ///
    /// @return True if the connection or file is open, otherwise false.
    virtual const bool isOpened() const { return isOpened_; }

    /// @brief Check if the connection or file has been initialized after opening.
    ///
    /// @return True if the connection or file has been initialized, otherwise false.
    virtual const bool isInitialized() const { return isInitialized_; }

protected:
    bool isInitialized_{false}; ///< Indicates whether the connection or file has been initialized.
    bool isOpened_{false};      ///< Indicates whether the connection or file is currently open.
    std::mutex ioMutex_;  ///< Mutex to synchronize access to the IO device
};

} // namespace platform::io
