#include "nav/can_client/CANDBCMessage.h"

namespace nav {
namespace can {

const std::string &CANDBCSignal::getName() const {
    return name_;
}

double CANDBCSignal::getValue() const {
    return value_;
}

uint64_t CANDBCSignal::getTimestampNanoSeconds() const {
    return timestampNanoSeconds;
}

void CANDBCSignal::setTimestampNanoSeconds(uint64_t timestampNanoSeconds) {
    CANDBCSignal::timestampNanoSeconds = timestampNanoSeconds;
}

void CANDBCSignal::setName(const std::string &name) {
    name_ = name;
}

void CANDBCSignal::setValue(double value) {
    value_ = value;
}

    const std::string &CANDBCMessage::getName() const {
    return name_;
}

uint32_t CANDBCMessage::getAddress() const {
        return address_;
}

const CANBus &CANDBCMessage::getCANBus() const {
    return canBus_;
}

const std::vector<CANDBCSignal> &CANDBCMessage::getSignals() const {
    return signals_;
}

void CANDBCMessage::setAddress(uint32_t address) {
    address_ = address;
}

void CANDBCMessage::setName(const std::string &name) {
    name_ = name;
}

CANBus CANDBCMessage::getCanBus() const {
    return canBus_;
}

void CANDBCMessage::setCanBus(CANBus canBus) {
    canBus_ = canBus;
}

void CANDBCMessage::setSignals(const std::vector<CANDBCSignal> &signals) {
    signals_ = signals;
}

} // namespace can
} // namespace nav