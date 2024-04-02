#pragma once

#include <string>
#include <optional>

namespace platform::io {
class IOInterface
{
public:
    virtual ~IOInterface() = default;
    virtual void write(const std::string& request) = 0;
    virtual std::optional<std::string> read(const std::string& request) = 0;
};

} // namespace av::devices
