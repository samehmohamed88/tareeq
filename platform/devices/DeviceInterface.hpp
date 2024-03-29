#pragma once

#include <variant>
#include <memory>

namespace platform::devices {

template<typename Error,
         typename ILogger>
class DeviceInterface {
public:

    DeviceInterface(std::shared_ptr<const ILogger> logger);

    /// TODO : add docs
    virtual std::variant<bool, Error> initialize() = 0;

    /// TODO : add docs
    virtual std::variant<bool, Error> close() = 0;

protected:
    std::shared_ptr<const ILogger> getLogger() const;
    std::shared_ptr<const ILogger> logger_;
};

template<typename Error, typename ILogger>
DeviceInterface<Error, ILogger>::DeviceInterface(std::shared_ptr<const ILogger> logger)
    : logger_{logger}
{}

template<typename Error, typename ILogger>
std::shared_ptr<const ILogger> DeviceInterface<Error, ILogger>::getLogger() const
{
    return logger_;
}

} // namespace platform::devices
