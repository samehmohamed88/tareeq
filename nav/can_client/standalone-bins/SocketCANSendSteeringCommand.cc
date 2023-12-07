
#include "nav/can_client/CANDBC.h"
#include "nav/can_client/CANDBCMessage.h"
#include "nav/can_client/CANClient.h"
#include "nav/can_client/SocketCANDevice.h"
#include "nav/can_client/SocketCANInterfaceImpl.h"
#include "nav/can_client/SocketCANMessage.h"

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
using namespace nav::can;

bool EXIT = false;

// dont send steering command right away
// but send everything else
bool DO_SEND = true;

int COUNTER = 1;

bool IN_CAR = true;


/// REMEMBER WE MAY STILL NEED TO CREATE SOME MORE FAKE MESSAGE
/// SUCH AS THE TESTER PRESENT MESSAGE
/// AND OTHER SYSTEM MESSAGES TO KEEP THE CARY HAPPY

void UpdateLKASStateMessage(CANDBCMessage &message) {
    for (auto signal : message.getSignals()) {
        if (signal.getName() == "LKAS_Alert_Msg") {
            signal.setValue(0);
        } else if (signal.getName() == "LKAS_Alert") {
            signal.setValue(0);
        } else if (signal.getName() == "LKAS_Alert_Msg") {
            signal.setValue(0);
        } else if (signal.getName() == "LKAS_ACTIVE") {
            signal.setValue(1);
        } else if (signal.getName() == "COUNTER") {
            signal.setValue(signal.getValue() + 1);
        }
    }
}

void UpdateESDistanceMessage(CANDBCMessage &message) {
    for (auto signal : message.getSignals()) {
        if (signal.getName() == "Cruise_Soft_Disable") {
            signal.setValue(0);
        } else if (signal.getName() == "Cruise_Fault") {
            signal.setValue(0);
        }
    }
}

std::vector<CANDBCMessage> ProcessMessages(std::vector<CANDBCMessage> &messages) {
    std::vector<CANDBCMessage> messageToSend{};
    for (auto &message : messages) {
        if (message.getName() == "ES_LKAS_State") {
            UpdateLKASStateMessage(message);
            CANDBCMessage msg = message;
            messageToSend.push_back(msg);
        } else if (message.getName() == "ES_Distance") {
            UpdateESDistanceMessage(message);
            CANDBCMessage msg = message;
            messageToSend.push_back(msg);
        }
    }
    return messageToSend;
}

bool CreateAndSendSteeringMessage(CANClient<SocketCANDevice<SocketCANInterfaceImpl>>& canDevice, bool positive) {
    std::vector<CANDBCSignal> signals{};

    signals.push_back(CANDBCSignal{"SET_1",1});

    if (positive) {
        signals.push_back(CANDBCSignal{"LKAS_Output",9.0});
        positive = false;
    } else {
        signals.push_back(CANDBCSignal{"LKAS_Output",-9.0});
        positive = true;
    }
    signals.push_back(CANDBCSignal{"LKAS_Request",1});

    CANDBCMessage message = CANDBCMessage{290,"ES_LKAS", can::CANBus::CAMERA_BUS, std::move(signals)};

    canDevice.sendMessage<SocketCANMessage>(message);
    return positive;
}

std::thread CreateReceiveThread(CANClient<SocketCANDevice<SocketCANInterfaceImpl>>& canDevice) {
    std::vector<uint8_t> chunck{};
    auto thread = std::thread([&]{
        while (!EXIT) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            canDevice.getMessages<SocketCANMessage>();
        }
    });
    return thread;
}

std::thread CreateSendThread(CANClient<SocketCANDevice<SocketCANInterfaceImpl>>& canDevice) {
    bool positive = true;
    auto thread = std::thread([&]{
        while (!EXIT) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            auto messages = canDevice.getQueuedMessagesAndClearQueue();
            auto messagesToSend = ProcessMessages(messages);
            ++COUNTER;
            if (COUNTER % 5 == 0 && DO_SEND) {
                // let's send a steering command
                positive = CreateAndSendSteeringMessage(canDevice, positive);
            }
            if (COUNTER % 500 == 0) {
                EXIT = true;
            }
        }
    });
    return thread;
}

int main() {

    auto canDBC = CANDBC::CreateInstance();

    auto socket = std::make_unique<SocketCANDevice<SocketCANInterfaceImpl>>(std::make_unique<SocketCANInterfaceImpl>("can0"));

    auto canDevice = CANClient<SocketCANDevice<SocketCANInterfaceImpl>>{std::move(socket), std::move(canDBC)};

    auto sendThread = CreateSendThread(canDevice);
    auto receiveThread = CreateReceiveThread(canDevice);

    sendThread.join();
    receiveThread.join();


    return 0;
}
