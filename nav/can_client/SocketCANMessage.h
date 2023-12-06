#pragma once

#include "nav/can_client/CANDBC.h"

#include <linux/can.h>

#include <cstring>
#include <exception>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

namespace nav {
namespace can {
using std::generic_category;

/// @brief A convenience wrapper for the Linux Socket CAN struct `can_frame`.
class SocketCANMessage {
public:
    SocketCANMessage(const struct can_frame frame) :
            canId_{frame.can_id}, frameData_(*frame.data, frame.can_dlc), rawFrame_{frame} {}

    SocketCANMessage(const uint32_t canId, const std::vector<uint8_t> frameData) : canId_(canId), frameData_(frameData) {
        if (frameData.size() > 8) {
            throw std::system_error(std::error_code(0xbadd1c, generic_category()), "Payload too big!");
        }

        struct can_frame rawFrame;
        rawFrame.can_id = canId;
        memcpy(rawFrame.data, frameData.data(), frameData.size());
        rawFrame.can_dlc = frameData.size();
        rawFrame_ = rawFrame;
    }

    virtual ~SocketCANMessage() {}

public:
    const bool isValid() const { return (rawFrame_.can_dlc != 0 && rawFrame_.can_id != 0); }
    const uint32_t getCanId() const { return canId_; }

    const std::vector<uint8_t> getFrameData() const { return frameData_; }

    const can_frame getRawFrame() const { return rawFrame_; }

    static SocketCANMessage fromCANDBCMessage(const CANDBCMessage &candbcMessage);

private:
    uint32_t canId_;
    std::vector<uint8_t> frameData_;
    struct can_frame rawFrame_;
};
} // namespace can
} // namespace nav
