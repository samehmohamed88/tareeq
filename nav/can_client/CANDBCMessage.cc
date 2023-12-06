#include "nav/can_client/CANDBCMessage.h"

namespace nav {
namespace can {

const std::string &CANDBCSignal::getName() const {
    return name_;
}

double CANDBCSignal::getValue() const {
    return value_;
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

} // namespace can
} // namespace nav