#pragma once

#include <string>
#include <memory>

namespace platform::devices {

template<
    typename SerialPortImpl,
    typename ILogger>
class SerialDevice
{
public:
    SerialDevice(std::shared_ptr<SerialPortImpl> serialPort, std::shared_ptr<ILogger> logger)
        : serialPort_{serialPort}
        , logger_{logger}
    {}

    void write(const std::string& data) { serialPort_.write(data); }

    std::string read() { return serialPort_.read(); }

private:
    std::shared_ptr<ILogger> logger_;
    std::shared_ptr<SerialPortImpl> serialPort_;
};

} // namespace platform::devices
