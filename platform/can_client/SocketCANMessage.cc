#include "can_client/SocketCANMessage.h"

#include "can_client/CANDBCMessage.h"

namespace nav {
namespace can {

const uint32_t SocketCANMessage::getCanId() const { return canId_; }
const std::vector<uint8_t> SocketCANMessage::getFrameData() const { return frameData_; }
const can_frame SocketCANMessage::getRawFrame() const { return rawFrame_; }


}
}