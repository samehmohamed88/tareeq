#include "nav/can_client/CANDBCMessageSchema.h"

namespace nav {
namespace can {

uint32_t CANDBCMessageSchema::getAddress() {
    return address_;
}

const std::string &CANDBCMessageSchema::getName() const {
    return name_;
}

uint32_t CANDBCMessageSchema::getSize() {
    return size_;
}

std::optional<std::reference_wrapper<const CANDBCSignalSchema>> CANDBCMessageSchema::getSignalSchemaByName(std::string signalName) const {
    const auto it = signalNameToSignalMap.find(signalName);
    if (it != signalNameToSignalMap.end()) {
        return std::cref(it->second);
    } else {
        return std::nullopt; // Represents an empty optional
    }
}
} // namespace can
} // namespace nav