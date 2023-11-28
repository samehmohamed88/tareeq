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
#include <cmath>

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
    /// the main CAN bus on which important messages are sent.  Messages such as steering torque, wheels speeds, etc.
    static constexpr int subaruMainCanBus = 0;
    /// TODO: we want this in an external config.
    static constexpr bool ignoreChecksum = true;
    /// TODO: we want this in an external config.
    static constexpr bool ignoreCounter = false;
    static constexpr int MAX_BAD_COUNTER = 5;
    static constexpr int CAN_INVALID_COUNT = 5;
    static constexpr size_t USB_TX_SOFT_LIMIT = 0x100U;
public:
    CommaAICANInterfaceWithBoostBuffer(std::unique_ptr<Device> device);

    /// Sets the safety model on the CAN Device
    /// The Comma AI CAN Device has a specific safety model where it starts in silent mode with No Output to prevent
    /// accidental movements/steering
    /// taken from https://github.com/commaai/cereal/blob/416c3d531c90ce16498d782bf383625a857ee74c/car.capnp#L567C8-L567C19
    bool setSafetyModel(SafetyModel safety_model, uint16_t safety_param=0U);

    uint8_t getHardwareType();
    bool receiveMessages(std::vector<uint8_t>& chunk);
    std::vector<uint8_t> sendMessages(const std::vector<CANMessage> &messages);
    std::vector<CANMessage> getCANMessagesAndClearContainer();

private:
    void addCANMessage(const CANMessage& message);

    std::string stripWhitespace(const std::string& input) {
        std::string output = input;
        output.erase(
                std::remove_if(output.begin(), output.end(), [](unsigned char ch) {
                    return std::isspace(ch);
                }),
                output.end()
        );
        return output;
    }

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

    void packValueUsingSignalSchema(std::vector<uint8_t> &writeBuffer, const CANDBC::SignalSchema &signalSchema, int64_t tmp_val) {
        int i = signalSchema.leastSignificantBit / 8;
        int bits = signalSchema.size;
        if (signalSchema.size < 64) {
            tmp_val &= ((1ULL << signalSchema.size) - 1);
        }

        while (i >= 0 && i < static_cast<int>(writeBuffer.size()) && bits > 0) {
            int shift = (int)(signalSchema.leastSignificantBit / 8) == i ? signalSchema.leastSignificantBit % 8 : 0;
            int size = std::min(bits, 8 - shift);

            writeBuffer[i] &= ~(((1ULL << size) - 1) << shift);
            writeBuffer[i] |= (tmp_val & ((1ULL << size) - 1)) << shift;

            bits -= size;
            tmp_val >>= size;
            i = signalSchema.isLittleEndian ? i+1 : i-1;
        }
    }

    bool updateCounter(CANFrame &dataFrame, int64_t v, int signalCountSize) {
        uint8_t old_counter = dataFrame.counter;
        dataFrame.counter = v;
        if (((old_counter+1) & ((1 << signalCountSize) -1)) != v) {
            dataFrame.numCounterErrors += 1;
            if (dataFrame.numCounterErrors > 1) {
                AERROR << "COUNTER FAIL "
                       << dataFrame.address
                       << " "
                       << dataFrame.numCounterErrors
                        << " "
                       <<  (int)v;
            }
            if (dataFrame.numCounterErrors >= MAX_BAD_COUNTER) {
                return false;
            }
        } else if (dataFrame.numCounterErrors > 0) {
            dataFrame.numCounterErrors--;
        }
        return true;
    }

private:
    mutable std::mutex mtx_;
    std::unique_ptr<Device> device_;
    std::vector<CANFrame> canDataFrames_;
    std::vector<CANMessage> canMessages;
    std::unique_ptr<CANDBC> canDatabase_;
    boost::circular_buffer<uint8_t> circularBuffer_;
    std::unique_ptr<std::vector<uint8_t>> transferBuffer_;
    std::unordered_map<uint32_t, uint32_t> messageCountsByAddress_;


    static constexpr size_t MAX_BUFFER_SIZE = 0x4000U; // Max buffer size
    static constexpr auto CAN_REJECTED_BUS_OFFSET = 0xC0U;
    static constexpr auto CAN_RETURNED_BUS_OFFSET = 0x80U;
};

template <class Device>
bool CommaAICANInterfaceWithBoostBuffer<Device>::setSafetyModel(SafetyModel safetyModel, uint16_t safetyParam) {
//    const uint8_t bmRequestType,
//    const uint8_t bRequest,
//    const uint16_t wValue,
//    const uint16_t wIndex,
//    std::vector<uint8_t> &data,
    std::vector<uint8_t> data{0};
//    data.push_back(safetyParam);
    DeviceStatus status = device_->controlWrite(Device::WriteRequest, static_cast<uint8_t>(DeviceRequests::SafetyModel), static_cast<uint16_t>(safetyModel), safetyParam, data);
    return status == DeviceStatus::SUCCESS;
}

template <class Device>
CommaAICANInterfaceWithBoostBuffer<Device>::CommaAICANInterfaceWithBoostBuffer(std::unique_ptr<Device> device) :
        mtx_{}
        , device_{std::move(device)}
        , canDataFrames_{}
        , canMessages{}
        , canDatabase_{CANDBC::CreateInstance()}
        , circularBuffer_{MAX_BUFFER_SIZE + getSizeOfCANHeader() + 64}
        , transferBuffer_{std::make_unique<std::vector<uint8_t>>(MAX_BUFFER_SIZE + getSizeOfCANHeader() + 64, 0)}
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
std::vector<uint8_t> CommaAICANInterfaceWithBoostBuffer<Device>::sendMessages(const std::vector<CANMessage> &messages) {
    // we fill up the `canBuffer` until the number of elements, i.e. position on the buffer
    // is greater than `USB_TX_SOFT_LIMIT`
    // then we send the buffer over the USB Interface to the Vehicle CAN
    uint bufferLength = 0;
    std::vector<uint8_t> canBuffer(2 * USB_TX_SOFT_LIMIT);

    // TODO: determine if flattening the maps will improve performance.
    // for example we can store std::unordered_map<std::pair(messageId, signalName), SignalSchema>
    // but this will require profiling to determine the bottlenecks.
    for (const auto& message : messages) {
        // set all values for all given signal/value pairs
        bool setCounterSignal = false;
        const auto messageSchemaRef = canDatabase_->getMessageByName(message.name);

        // we lookup message schema by name and make sure we don't get a nullptr
        if (messageSchemaRef.has_value()) {
            // get the MessageSchema from the std::optional and begin processing message
            const auto& messageSchema = messageSchemaRef.value().get();

            // first we make a std::vector to hold the raw data that will be transferred over USB
            std::vector<uint8_t> rawData(messageSchema.size, 0);

            for (const auto& messageSignal : message.signals) {
                // we lookup the Signal Schema by name inside the MessageSchema and make sure we don't get a nullptr
                const auto signalSchemaRef = messageSchema.getSignalSchmeByName(messageSignal.name);
                if (signalSchemaRef.has_value()) {
                    // get the Signal Schema from std::optional
                    const auto signalSchema = signalSchemaRef.value().get();

                    // convert value to raw value using signal schema offset and factor
                    int64_t tmp_val = (int64_t)(std::round((messageSignal.value - signalSchema.offset) / signalSchema.factor));
                    if (tmp_val < 0) {
                        tmp_val = (1ULL << signalSchema.size) + tmp_val;
                    }
                    packValueUsingSignalSchema(rawData, signalSchema, tmp_val);

                    setCounterSignal = setCounterSignal || (signalSchema.name == "COUNTER");
                    if (setCounterSignal) {
                        messageCountsByAddress_[message.address] = messageSignal.value;
                    }
                }
            }

            // set message counter
            // we lookup the COUNT SignalSchema by name inside the MessageSchema and make sure we don't get a nullptr
            const auto countSignalSchemaRef = messageSchema.getSignalSchmeByName("COUNT");
            // setCounterSignal false means that the CANMessage above did not supply a COUNT Signal
            // for example ES_LKAS message does not supply COUNT in code so we set it here
            if (!setCounterSignal && countSignalSchemaRef.has_value()) {
                const auto& countSignalSchema = countSignalSchemaRef.value().get();

                // we zero out the counter first if it exists
                if (messageCountsByAddress_.find(message.address) == messageCountsByAddress_.end()) {
                    messageCountsByAddress_[message.address] = 0;
                }
                packValueUsingSignalSchema(rawData, countSignalSchema, messageCountsByAddress_[message.address]);
                messageCountsByAddress_[message.address] = (messageCountsByAddress_[message.address] + 1) % (1 << countSignalSchema.size);
            }

            // set message checksum
            // we lookup the CHECKSUM SignalSchema by name inside the MessageSchema and make sure we don't get a nullptr
            const auto checksumSignalSchemaRef = messageSchema.getSignalSchmeByName("CHECKSUM");
            if (checksumSignalSchemaRef.has_value()) {
                const auto& checksumSignalSchema = checksumSignalSchemaRef.value().get();

                // use the Subaru checksum calculation copies from Comma AI
                unsigned int checksum = checksumSignalSchema.calcSubaruChecksum(message.address, rawData);

                // pack the checksum into the output buffer
                packValueUsingSignalSchema(rawData, checksumSignalSchema, checksum);
            }

            uint8_t dataLengthCode = CANDBC::bufferSizeToDataLengthCode(rawData.size());
            assert(rawData.size() <= 64);
            // assert we did the converstion back successfully
            assert(rawData.size() == CANDBC::dataLengthCodeToNumBytes[dataLengthCode]);

            CANHeader canHeader{};
            canHeader.addr = message.address;
            canHeader.extended = (message.address >= 0x800) ? 1 : 0;
            canHeader.dataLengthCode = dataLengthCode;
            canHeader.bus = subaruMainCanBus;
            canHeader.checksum = 0;

            memcpy(&canBuffer[bufferLength], &canHeader, sizeof(CANHeader));
            memcpy(&canBuffer[bufferLength + sizeof(CANHeader)], rawData.data(), rawData.size());
            uint32_t msg_size = sizeof(CANHeader) + rawData.size();

            // set checksum
            ((CANHeader *) &canBuffer[bufferLength])->checksum = calculate_checksum(&canBuffer[bufferLength], msg_size);

            bufferLength += msg_size;
            if (bufferLength >= USB_TX_SOFT_LIMIT) {
                std::vector<uint8_t> transferData;
                transferData.insert(
                        transferData.end(),
                        std::make_move_iterator(canBuffer.begin()),
                        std::make_move_iterator(canBuffer.begin() + bufferLength)
                );
                device_->bulkWrite(
                        static_cast<uint8_t>(DeviceRequests::WRITE_TO_CAN_BUS),
                        transferData,
                        std::make_unique<int>(0));
                bufferLength = 0;
                return transferData;
            }
        }
    }
    // send remaining packets
    if (bufferLength > 0) {
        std::vector<uint8_t> transferData;
        transferData.insert(
                transferData.end(),
                std::make_move_iterator(canBuffer.begin()),
                std::make_move_iterator(canBuffer.begin() + bufferLength)
        );
        device_->bulkWrite(
                static_cast<uint8_t>(DeviceRequests::WRITE_TO_CAN_BUS),
                transferData,
                std::make_unique<int>(0));
        return transferData;
    }
//    return true;
    return std::vector<uint8_t>{};
}

template <class Device>
std::vector<CANMessage> CommaAICANInterfaceWithBoostBuffer<Device>::getCANMessagesAndClearContainer() {
    std::lock_guard<std::mutex> lock(mtx_);
    // Move the entire vector
    std::vector<CANMessage> currentMessages = std::move(canMessages);
    // After the move, canMessages is empty
    // No need to call clear()
    return currentMessages;
}

template <class Device>
bool CommaAICANInterfaceWithBoostBuffer<Device>::receiveMessages(std::vector<uint8_t>& chunk1) {
    std::shared_ptr<int> transferred = std::make_shared<int>(0);
    device_->bulkRead(
            static_cast<uint8_t>(DeviceRequests::READ_CAN_BUS),
            *transferBuffer_.get(),
            transferred);
     //this actual number of bytes transferred from USB
     //let's dereference once so we can reuse it in multiple checks
     int received = *transferred;

//    if (!device_.isCommHealthy()) {
//        return false;
//    }

//    int received = chunk.size();

    // Check if adding new data exceeds max buffer size
    if (circularBuffer_.size() + received > MAX_BUFFER_SIZE) {
        // Handle buffer overflow, e.g., log error, discard data, etc.
        AWARN << "Exceeding maximum buffer size, discarding data";
        return false;
    }

    // Add all received bytes to the deque
    circularBuffer_.insert(circularBuffer_.end(), transferBuffer_->begin(), transferBuffer_->begin() + received);

    while (circularBuffer_.size() >= getSizeOfCANHeader()) {
        // the front of the buffer should always be a can header
        CANHeader canHeader;
        memcpy(&canHeader, circularBuffer_.array_one().first, getSizeOfCANHeader());

        // get the message length from the header
        if (
                (canHeader.dataLengthCode < 0 ||
                        canHeader.dataLengthCode > getSizeOfDataLengthArray() / getSizeOfDataLengthElement())
                || (canHeader.bus != subaruMainCanBus)
                ) {
            AERROR << "Message has invalid Data Length of greater than 64 bits,"
                   << "or sent on wrong CAN Bus.  Erasing buffer";
            circularBuffer_.erase(circularBuffer_.begin(), circularBuffer_.end());
            break;
        }
        const uint8_t dataLength = CANDBC::dataLengthCodeToNumBytes[canHeader.dataLengthCode];

        if (circularBuffer_.size() < getSizeOfCANHeader() + dataLength) {
            // we don't have all the data for this message yet
            // so we leave the data on the buffer and the next iteration of receiveMessages() should
            // append the remainder of the message to the buffer
            break;
        }

        // create a CANFrame object to hold the raw message data
        CANFrame &canFrame = canDataFrames_.emplace_back();
        // Set canData properties
        canFrame.busTime = 0;
        canFrame.address = canHeader.addr;
        canFrame.src = canHeader.bus;
        if (canHeader.rejected) {
            canFrame.src += CAN_REJECTED_BUS_OFFSET;
        }
        if (canHeader.returned) {
            canFrame.src += CAN_RETURNED_BUS_OFFSET;
        }
        if (calculate_checksum(transferBuffer_->data(), getSizeOfCANHeader() + dataLength) != 0) {
            // checksum did not pass, so we clear the header and message from the buffer
            circularBuffer_.erase(circularBuffer_.begin(), circularBuffer_.end());
            AINFO << "Panda CAN checksum failed";
            break;
        }

        // we move the raw message from the buffer to the CANFrame object's data vector
        // and erase the header and message from the buffer
        canFrame.data = std::move(std::vector<uint8_t>{
            circularBuffer_.begin() + getSizeOfCANHeader(),
            circularBuffer_.begin() + getSizeOfCANHeader() + dataLength});

        // now we can erase the entire message form the buffer
        circularBuffer_.erase(circularBuffer_.begin(), circularBuffer_.begin() + getSizeOfCANHeader() + dataLength);
    }

    for (auto &dataFrame : canDataFrames_) {
        CANMessage canMessage;
        canMessage.address = dataFrame.address;

        auto const &signalsRef = canDatabase_->getSignalSchemasByAddress(dataFrame.address);
        if (signalsRef.has_value()) {
            auto const &signals = signalsRef.value().get();
            canMessage.signals.reserve(signals.size());
            canMessage.name = signals[0].messageName;

            for (auto const &signalSchema: signals) {
                int64_t tmp = parseValueUsingSignalSchema(dataFrame.data, signalSchema);

                if (signalSchema.is_signed) {
                    tmp -= ((tmp >> (signalSchema.size - 1)) & 0x1) ? (1ULL << signalSchema.size) : 0;
                }

//                AINFO << "parse 0x%X %s -> %ld\n" << address <<  sig.name, tmp);
                bool checksum_failed = false;
                if (!ignoreChecksum) {
                    if (signalSchema.calcSubaruChecksum(dataFrame.address, dataFrame.data) != tmp) {
                        checksum_failed = true;
                    }
                }
                bool counter_failed = false;
                if (!ignoreCounter) {
                    if (signalSchema.type == CANDBC::SignalType::COUNTER) {
                        counter_failed = !updateCounter(dataFrame, tmp, signalSchema.size);
                    }
                }
                if (checksum_failed || counter_failed) {
                    AERROR << "Message checks failed: "
                           << dataFrame.address
                           << " checksum failed " << checksum_failed
                           << " counter failed " << counter_failed;
                    continue;
                }
                auto &parsedSignal = canMessage.signals.emplace_back();
                parsedSignal.value = tmp * signalSchema.factor + signalSchema.offset;
                parsedSignal.name = signalSchema.name;
            }
            // we add the can message to the CAN Message Queue which is picked up by the CAN Timer Component
            // on a specified interval, converted to DDS messages and cleared.
            addCANMessage(std::move(canMessage));
        }
    }
    // we processed all messages, so let's clear for the next iteration
    canDataFrames_.clear();
    return true;
}

template <class Device>
void CommaAICANInterfaceWithBoostBuffer<Device>::addCANMessage(const CANMessage& message) {
    std::lock_guard<std::mutex> lock(mtx_);
    canMessages.push_back(message);
}

} // namespace can
} // namespace nav