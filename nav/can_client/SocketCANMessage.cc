#include "nav/can_client/SocketCANMessage.h"
#include "nav/can_client/CANDBCMessage.h"

namespace nav {
namespace can {

static SocketCANMessage fromCANDBCMessage(const CANDBCMessage &canDBCMessage, std::vector<uint8_t> frameData_) {
    SocketCANMessage message{canDBCMessage.getAddress(), frameData_};
    return message;
}

const uint32_t SocketCANMessage::getCanId() const { return canId_; }
const std::vector<uint8_t> SocketCANMessage::getFrameData() const { return frameData_; }
const can_frame SocketCANMessage::getRawFrame() const { return rawFrame_; }


}
}