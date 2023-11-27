#pragma once

#include "nav/can_client/CANDBC.h"
#include "nav/can_client/DeviceInfo.h"
#include "cyber/common/log.h"

#include <boost/circular_buffer.hpp>

#include <vector>
#include <memory>
#include <fstream>
#include <iterator>
#include <string>
#include <functional>
#include <iostream>

namespace nav {
namespace can {

template <class Device>
class CommaAICANInterfaceWithBoostBuffer {
public:
    /// the Comma AI Red Panda Vendor ID
    static constexpr uint16_t vendorID_ = 0xbbaa;
    /// thee Comma AI Red Panda Product ID
    static constexpr uint16_t productID_ = 0xddcc;
    /// interface number taken from https://github.com/commaai/openpilot/blob/master/selfdrive/boardd/panda_comms.cc#L73
    static constexpr int interfaceNumber_ = 0;
public:
    CommaAICANInterfaceWithBoostBuffer(std::unique_ptr<Device> device);

    /// Sets the safety model on the CAN Device
    /// The Comma AI CAN Device has a specific safety model where it starts in silent mode with No Output to prevent
    /// accidental movements/steering
    /// taken from https://github.com/commaai/cereal/blob/416c3d531c90ce16498d782bf383625a857ee74c/car.capnp#L567C8-L567C19
    bool setSafetyModel(SafetyModel safety_model, uint16_t safety_param=0U);

    uint8_t getHardwareType();
    bool receiveMessages(std::vector<uint8_t>& chunk);
    std::vector<CANMessage> getCANMessagesAndClearContainer();

private:
    void addCANMessage(const CANMessage& message);

    uint8_t calculate_checksum(const uint8_t *data, uint32_t len) {
        uint8_t checksum = 0U;
        for (uint32_t i = 0U; i < len; i++) {
            checksum ^= data[i];
        }
        return checksum;
    }

    int64_t parseValueUsingSignalSchema(const std::vector<uint8_t> &messageData, const CANDBC::SignalSchema signalSchema) {
        int64_t ret = 0;
        int i = signalSchema.mostSignificantBit / 8;
        int bits = signalSchema.size;
        while (i >= 0 && i < static_cast<int>(messageData.size()) && bits > 0) {
            int lsb = (int)(signalSchema.leastSignificantBit / 8) == i ? signalSchema.leastSignificantBit : i*8;
            int msb = (int)(signalSchema.mostSignificantBit / 8) == i ? signalSchema.mostSignificantBit : (i+1)*8 - 1;
            int size = msb - lsb + 1;

            uint64_t d = (messageData[i] >> (lsb - (i*8))) & ((1ULL << size) - 1);
            ret |= d << (bits - size);

            bits -= size;
            i = signalSchema.isLittleEndian ? i-1 : i+1;
        }
        return ret;
    }

private:
    mutable std::mutex mtx;
    std::unique_ptr<Device> device_;
    std::vector<CANFrame> canDataFrames_;
    std::vector<CANMessage> canMessages;
    std::unique_ptr<CANDBC> canDatabase_;
    boost::circular_buffer<uint8_t> circularBuffer;
    std::unique_ptr<std::vector<uint8_t>> transferBuffer;


    static constexpr size_t MAX_BUFFER_SIZE = 0x4000U; // Max buffer size
    static constexpr auto CAN_REJECTED_BUS_OFFSET = 0xC0U;
    static constexpr auto CAN_RETURNED_BUS_OFFSET = 0x80U;
};

template <class T>
bool CommaAICANInterfaceWithBoostBuffer<T>::setSafetyModel(SafetyModel safetyModel, uint16_t safetyParam) {
    DeviceStatus status = device_.controlWrite(DeviceRequests::SafetyModel, static_cast<uint16_t>(safetyModel), safetyParam);
    return status == DeviceStatus::SUCCESS;
}

template <class Device>
CommaAICANInterfaceWithBoostBuffer<Device>::CommaAICANInterfaceWithBoostBuffer(std::unique_ptr<Device> device) :
        mtx{}
        , device_{std::move(device)}
        , canDataFrames_{}
        , canMessages{}
        , canDatabase_{CANDBC::CreateInstance()}
        , circularBuffer{MAX_BUFFER_SIZE + sizeof(CANHeader) + 64}
        , transferBuffer{std::make_unique<std::vector<uint8_t>>(MAX_BUFFER_SIZE + sizeof(CANHeader) + 64, 0)}
        {}

template <class Device>
uint8_t CommaAICANInterfaceWithBoostBuffer<Device>::getHardwareType() {
    std::vector<uint8_t> data{0};
    // This is used to pass a parameter to the device, specific to the request, in this case 0
    uint16_t requestValue = 0;
    // The index field for the setup packet, this is often 0
    uint16_t requestIndex = 0;
    DeviceStatus status = device_->controlRead(
            Device::ReadRequest,
            static_cast<uint8_t>(DeviceRequests::HardwareType),
            requestValue,
            requestIndex,
            data);
    if (status == DeviceStatus::SUCCESS) {
        return data[0];
    }
    return 200;
}

template <class Device>
std::vector<CANMessage> CommaAICANInterfaceWithBoostBuffer<Device>::getCANMessagesAndClearContainer() {
    std::lock_guard<std::mutex> lock(mtx);
    // Move the entire vector
    std::vector<CANMessage> currentMessages = std::move(canMessages);
    // After the move, canMessages is empty
    // No need to call clear()
    return currentMessages;
}

template <class Device>
bool CommaAICANInterfaceWithBoostBuffer<Device>::receiveMessages(std::vector<uint8_t>& chunk) {
//    std::shared_ptr<int> transferred = std::make_shared<int>(0);
//    device_->bulkRead(
//            static_cast<uint8_t>(DeviceRequests::READ_CAN_BUS),
//            *temp_buffer.get(),
//            transferred);
    // this actual number of bytes transferred from USB
    // let's dereference once so we can reuse it in multiple checks
//    int received = *transferred;

//    if (!device_.isCommHealthy()) {
//        return false;
//    }
    bool ignore_checksum = false;
    bool ignore_counter = false;
    int received = chunk.size();

    // Check if adding new data exceeds max buffer size
    if (circularBuffer.size() + received > MAX_BUFFER_SIZE) {
        // Handle buffer overflow, e.g., log error, discard data, etc.
        AWARN << "Exceeding maximum buffer size, discarding data";
        return false;
    }

    // Add all received bytes to the deque
    circularBuffer.insert(circularBuffer.end(), chunk.begin(), chunk.begin() + received);

    while (circularBuffer.size() >= sizeof(CANHeader)) {
        // the front of the buffer should always be a can header
        CANHeader canHeader;
        memcpy(&canHeader, circularBuffer.array_one().first, sizeof(CANHeader));

        // get the message length from the header
        const uint8_t dataLength = CANDBC::dataLengthCodeToNumBytes[canHeader.data_len_code];
        if (circularBuffer.size() < sizeof(CANHeader) + dataLength) {
            // we don't have all the data for this message yet
            // so we leave the data on the buffer and the next iteration of receiveMessages() should
            // append the remainder of the message to the buffer
            break;
        }

        // create a CANFrame object to hold the raw message data
        uint32_t bus_offset = 0;
        CANFrame &canFrame = canDataFrames_.emplace_back();
        // Set canData properties
        canFrame.busTime = 0;
        canFrame.address = canHeader.addr;
        canFrame.src = canHeader.bus + bus_offset;
        if (canHeader.rejected) {
            canFrame.src += CAN_REJECTED_BUS_OFFSET;
        }
        if (canHeader.returned) {
            canFrame.src += CAN_RETURNED_BUS_OFFSET;
        }
        if (calculate_checksum(chunk.data(), sizeof(CANHeader) + dataLength) != 0) {
            // checksum did not pass, so we clear the header and message from the buffer
            circularBuffer.erase(circularBuffer.begin(), circularBuffer.end());
            AINFO << "Panda CAN checksum failed";
            break;
        }

        // we move the raw message from the buffer to the CANFrame object's data vector
        // and erase the header and message from the buffer
        canFrame.data = std::move(std::vector<uint8_t>{
            circularBuffer.begin() + sizeof(CANHeader),
            circularBuffer.begin() + sizeof(CANHeader) + dataLength});

        // now we can erase the entire message form the buffer
        circularBuffer.erase(circularBuffer.begin(), circularBuffer.begin() + sizeof(CANHeader) + dataLength);
    }

    for (auto const& dataFrame : canDataFrames_) {
        CANMessage canMessage;
        canMessage.address = dataFrame.address;

        auto const &signalsRef = canDatabase_->getSignalSchemasByAddress(dataFrame.address);
        if (signalsRef.has_value()) {

            auto const &signals = signalsRef.value().get();
            canMessage.signals.reserve(signals.size());
            canMessage.name = signals[0].messageName;


            for (auto const &signalSchema: signals) {
                auto &parsedSignal = canMessage.signals.emplace_back();
                int64_t tmp = parseValueUsingSignalSchema(dataFrame.data, signalSchema);
                if (signalSchema.is_signed) {
                    tmp -= ((tmp >> (signalSchema.size - 1)) & 0x1) ? (1ULL << signalSchema.size) : 0;
                }

//                AINFO << "parse 0x%X %s -> %ld\n" << address <<  sig.name, tmp);
                bool checksum_failed = false;
                if (!ignore_checksum) {
                    if (signalSchema.calcChecksum != nullptr && signalSchema.calcChecksum(address, sig, dat) != tmp) {
                        checksum_failed = true;
                    }
                }
                bool counter_failed = false;
                if (!ignore_counter) {
                    if (signalSchema.type == SignalType::COUNTER) {
                        counter_failed = !update_counter_generic(tmp, sig.size);
                    }
                }
                if (checksum_failed || counter_failed) {
                    LOGE("0x%X message checks failed, checksum failed %d, counter failed %d", address, checksum_failed, counter_failed);
                    return false;
                }
                parsedSignal.value = tmp * signalSchema.factor + signalSchema.offset;
                parsedSignal.name = signalSchema.name;
                if (canMessage.name == "Steering_Torque") {
                    if (parsedSignal.name == "Steering_Torque" && parsedSignal.value > 0) {
                        std::cout << ">>>>>>>>>>>>> HAPPY " << parsedSignal.value;
                    }
                }
//                all_vals[i].push_back(vals[i]);
            }
        }
        // we add the can message to the CAN Message Queue which is picked up by the CAN Timer Component
        // on a specified interval, converted to DDS messages and cleared.
        addCANMessage(std::move(canMessage));
    }
    // we processed all messages, so let's clear for the next iteration
    canDataFrames_.clear();
    return true;
}

template <class Device>
void CommaAICANInterfaceWithBoostBuffer<Device>::addCANMessage(const CANMessage& message) {
    std::lock_guard<std::mutex> lock(mtx);
    canMessages.push_back(message);
}

} // namespace can
} // namespace nav