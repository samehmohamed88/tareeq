#include "can_client/CANDBCMessageSchema.h"

namespace nav {
namespace can {

uint32_t CANDBCMessageSchema::getAddress() {
    return address_;
}

void CANDBCMessageSchema::setAddress(uint32_t address) {
    address_ = address;
}

const std::string &CANDBCMessageSchema::getName() const {
    return name_;
}

void CANDBCMessageSchema::setName(std::string name) {
    name_ = name;
}

uint32_t CANDBCMessageSchema::getSize() const {
    return size_;
}

void CANDBCMessageSchema::setSize(uint32_t size) {
    size_ = size;
}

std::optional<std::reference_wrapper<const CANDBCSignalSchema>> CANDBCMessageSchema::getSignalSchemaByName(std::string signalName) const {
    const auto it = signalNameToSignalMap_.find(signalName);
    if (it != signalNameToSignalMap_.end()) {
        return std::cref(it->second);
    } else {
        return std::nullopt; // Represents an empty optional
    }
}

const std::vector<CANDBCSignalSchema> &CANDBCMessageSchema::getSignals() const {
    return signals_;
}

void CANDBCMessageSchema::setSignals(const std::vector<CANDBCSignalSchema> &signals) {
    signals_ = signals;
}

void CANDBCMessageSchema::moveSignalSchemaToMap(std::string signalSchemaName, CANDBCSignalSchema signalSchema) {
    signalNameToSignalMap_.emplace(std::move(signalSchemaName), std::move(signalSchema));
}

const std::unordered_map<std::string, const CANDBCSignalSchema> &
CANDBCMessageSchema::getSignalNameToSignalMap() const {
    return signalNameToSignalMap_;
}

void CANDBCMessageSchema::setSignalNameToSignalMap(
        const std::unordered_map<std::string, const CANDBCSignalSchema> &signalNameToSignalMap) {
    CANDBCMessageSchema::signalNameToSignalMap_ = signalNameToSignalMap;
}
} // namespace can
} // namespace nav