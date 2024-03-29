#pragma once

#include <string>
#include <functional>

namespace platform::devices {
class ISerialPort
{
public:
    using ReadCallback = std::function<void(const std::string&)>;
    virtual ~ISerialPort() = default;
    virtual void write(const std::string& data) = 0;
    virtual void read(const ReadCallback& callback) = 0;
};

} // namespace av::devices
