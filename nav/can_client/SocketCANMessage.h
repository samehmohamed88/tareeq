/**
 * @file CANMessage.hpp
 * @author Simon Cahill (simonc@online.de)
 * @brief Contains the implementation of a CAN message representation in C++.
 * @version 0.1
 * @date 2020-07-01
 * 
 * @copyright Copyright (c) 2020 Simon Cahill
 *
 *  Copyright 2020 Simon Cahill
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#pragma once

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
class CANMessage {
public:
    CANMessage(const struct can_frame frame) :
            canId_{frame.can_id}, frameData_(*frame.data, frame.can_dlc), rawFrame_{frame} {}

    CANMessage(const uint32_t canId, const std::vector<uint8_t> frameData) : canId_(canId), frameData_(frameData) {
        if (frameData.size() > 8) {
            throw std::system_error(std::error_code(0xbadd1c, generic_category()), "Payload too big!");
        }

        struct can_frame rawFrame;
        rawFrame.can_id = canId;
        memcpy(rawFrame.data, frameData.data(), frameData.size());
        rawFrame.can_dlc = frameData.size();
        rawFrame_ = rawFrame;
    }

    virtual ~CANMessage() {}

public:
    const bool isValid() const { return (rawFrame_.can_dlc != 0 && rawFrame_.can_id != 0); }
    const uint32_t getCanId() const { return canId_; }

    const std::vector<uint8_t> getFrameData() const { return frameData_; }

    const can_frame getRawFrame() const { return rawFrame_; }

private:
    uint32_t canId_;
    std::vector<uint8_t> frameData_;
    struct can_frame rawFrame_;
};
} // namespace can
} // namespace nav
