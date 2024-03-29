#pragma once

#include <string>

namespace av::devices {
class ISerialPort
{
public:
    virtual ~ISerialPort() = default;
    virtual void write(const std::string& data) = 0;
    virtual std::string read() = 0;
};

} // namespace av::devices
