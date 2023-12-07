#pragma once

#include "nav/can_client/SocketCANInterface.h"

#include <string>
#include <mutex>
#include <memory>

namespace nav {
namespace can {

class SocketCANInterfaceImpl : public SocketCANInterface {
public:
    SocketCANInterfaceImpl(std::string interfaceName);
    void initDevice();
    ssize_t readSocket(void *buf, size_t count) const override;
    ssize_t writeToSocket(const void *buf, size_t count) const override;
    int close(int fd) const override;
private:
    /// The bit mask used to filter CAN messages
    int32_t canFilterMask_;
    /// The protocol used when communicating via CAN
    int32_t canProtocol_;
    /// The CAN socket file descriptor
    int32_t socketFd_;
    /// The CAN interface used for communication (e.g. can0, can1, ...)
    std::string interfaceName_;
};

/**
* @brief Formats a std string object.
*
* @remarks Yoinked from https://github.com/Beatsleigher/liblogpp :)
*
* @tparam Args The formatting argument types.
* @param format The format string.
* @param args The format arguments (strings must be converted to C-style strings!)
*
* @return string The formatted string.
*/
template<typename... Args>
std::string formatString(const std::string& format, Args... args)  {
    auto stringSize = snprintf(NULL, 0, format.c_str(), args...) + 1; // +1 for \0
    std::unique_ptr<char[]> buffer(new char[stringSize]);
    snprintf(buffer.get(), stringSize, format.c_str(), args...);
    return std::string(buffer.get(), buffer.get() + stringSize - 1); // std::string handles termination for us.
}

} // namespace can
} // namespace nav