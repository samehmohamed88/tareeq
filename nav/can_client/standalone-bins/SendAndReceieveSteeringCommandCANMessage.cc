
#include "nav/can_client/CANDBC.h"
#include "nav/can_client/UsbDevice.h"
#include "nav/can_client/LibUsbDevice.h"
#include "nav/can_client/CommaAICANInterfaceWithBoostBuffer.h"

#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <libusb-1.0/libusb.h>

#include <fstream>
#include <chrono>
#include <thread>
#include <vector>
#include <iostream>
#include <cstdint>  // For uint8_t
#include <unordered_map>

using namespace nav;

bool EXIT = false;

// dont send steering command right away
// but send everything else
bool DO_SEND = false;

int COUNTER = 0;

void UpdateLKASStateMessage(can::CANMessage &message) {
    for (auto signal : message.signals) {
        if (signal.name == "LKAS_Alert_Msg") {
            signal.value = 0;
        } else if (signal.name == "LKAS_Alert") {
            signal.value = 0;
        } else if (signal.name == "LKAS_Alert_Msg") {
            signal.value = 0;
        } else if (signal.name == "LKAS_ACTIVE") {
            signal.value = 1;
        } else if (signal.name == "COUNTER") {
            signal.value++;
        }
    }
}

void UpdateESDistanceMessage(can::CANMessage &message) {
    for (auto signal : message.signals) {
        if (signal.name == "Cruise_Soft_Disable") {
            signal.value = 0;
        } else if (signal.name == "Cruise_Fault") {
            signal.value = 0;
        }
    }
}

std::vector<can::CANMessage> ProcessMessages(std::vector<can::CANMessage> &messages) {
    std::vector<can::CANMessage> messageToSend{};
    for (auto &message : messages) {
        if (message.name == "ES_LKAS_State") {
            UpdateLKASStateMessage(message);
            can::CANMessage msg = message;
            messageToSend.push_back(msg);
        } else if (message.name == "ES_Distance") {
            UpdateESDistanceMessage(message);
            can::CANMessage msg = message;
            messageToSend.push_back(msg);
        }
    }
    return messageToSend;
}

void CreateAndSendSteeringMessage(can::CommaAICANInterfaceWithBoostBuffer<can::UsbDevice<can::LibUsbDevice>>& canDevice) {
    std::vector<can::CANMessage::CANSignal> signals{};
    signals.push_back(can::CANMessage::CANSignal{
                              0,
                              "SET_1",
                              1});
    signals.push_back(can::CANMessage::CANSignal{
                              0,
                              "LKAS_Output",
                              9.0});
    signals.push_back(can::CANMessage::CANSignal{
            0,
            "LKAS_Request",
            1});
    can::CANMessage message = can::CANMessage{290,"ES_LKAS",std::move(signals)};
    std::vector<uint8_t> sentData = canDevice.sendMessages(std::vector<can::CANMessage>{message});
}

std::thread CreateReceiveThread(can::CommaAICANInterfaceWithBoostBuffer<can::UsbDevice<can::LibUsbDevice>>& canDevice) {
    std::vector<uint8_t> chunck{};
    auto thread = std::thread([&]{
        while (!EXIT) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            canDevice.receiveMessages(chunck, true);
        }
    });
    return thread;
}

std::thread CreateSendThread(can::CommaAICANInterfaceWithBoostBuffer<can::UsbDevice<can::LibUsbDevice>>& canDevice) {
    std::vector<uint8_t> chunck{};
    auto thread = std::thread([&]{
        while (!EXIT) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            auto messages = canDevice.getCANMessagesAndClearContainer();
            auto messagesToSend = ProcessMessages(messages);
            canDevice.sendMessages(messages);
            ++COUNTER;
            if (COUNTER % 5 == 0 && DO_SEND) {
                // let's send a steering command
                CreateAndSendSteeringMessage(canDevice);
            }
        }
    });
    return thread;
}



int main() {
    using namespace nav;
    static constexpr uint16_t vendorID_ = 0xbbaa;
    static constexpr uint16_t productID_ = 0xddcc;


//    std::unordered_map<std::string , int32_t > messageMap{};
//
    auto device = std::make_unique<can::UsbDevice<can::LibUsbDevice>>(
            std::make_unique<can::LibUsbDevice>(),
                    vendorID_,
                    productID_,
                    0);

    auto canDevice = can::CommaAICANInterfaceWithBoostBuffer<can::UsbDevice<can::LibUsbDevice>>{std::move(device)};
    auto hw = canDevice.getHardwareType();
    assert(hw == 7);
    std::cout << "Hardware Type == " << std::to_string(hw) << std::endl;
    bool valid = canDevice.setSafetyModel(can::SafetyModel::AllOutput, 1);
    std::cout << "Safety Model set " << valid << std::endl;

    auto sendThread = CreateSendThread(canDevice);
    auto receiveThread = CreateReceiveThread(canDevice);

    sendThread.join();
    receiveThread.join();

    if (COUNTER % 10000 == 0) {
        EXIT = true;
    }

    return 0;
}
