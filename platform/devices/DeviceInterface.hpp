#pragma once

#include <variant>
#include <memory>

namespace platform::devices {

template<typename Error,
         typename DeviceManager,
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
private:
    std::shared_ptr<const ILogger> logger_;
};
template<typename Error, typename DeviceManager, typename ILogger>
std::shared_ptr<const ILogger> DeviceInterface<Error, DeviceManager, ILogger>::getLogger() const
{
    return logger_;
}

template<typename Error, typename DeviceManager, typename ILogger>
DeviceInterface<Error, DeviceManager, ILogger>::DeviceInterface(std::shared_ptr<const ILogger> logger)
    : logger_{logger} {}

} // namespace platform::devices
