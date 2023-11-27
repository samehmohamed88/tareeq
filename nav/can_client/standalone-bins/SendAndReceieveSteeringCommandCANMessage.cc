
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

int main() {
    using namespace nav;
    static constexpr uint16_t vendorID_ = 0xbbaa;
    static constexpr uint16_t productID_ = 0xddcc;

    std::unordered_map<std::string , int32_t > messageMap{};

    auto device = std::make_unique<can::UsbDevice<can::LibUsbDevice>>(
            std::make_unique<can::LibUsbDevice>(),
                    vendorID_,
                    productID_,
                    0);

    auto canDevice = can::CommaAICANInterfaceWithBoostBuffer<can::UsbDevice<can::LibUsbDevice>>{std::move(device)};
    auto hw = canDevice.getHardwareType();
    assert(hw == 7);
    std::cout << "Hardware Type == " << std::to_string(hw) << std::endl;


//    auto thread1 = std::thread([&]{
//        while (true) {
//            auto messages = canDevice.getCANMessagesAndClearContainer();
////            std::this_thread::sleep_for(std::chrono::milliseconds(100));
////            std::cout << " size of messages " << messages.size() << std::endl;
//            for (auto const& message : messages) {
////                std::cout << "Message name is "
////                          << message.name
////                          << " and address is "
////                          << message.address
////                          << " and signal size "
////                          << message.signals.size() << std::endl;
//
//                if (messageMap.find(message.name) != messageMap.end()) {
//                    messageMap[message.name]++;
//                } else {
//                    messageMap[message.name] = 1;
//                }
//                for (auto const& signal : message.signals) {
//                    if (signal.name == "CHECKSUM" || signal.name == "COUNTER") {
//                        continue;
//                    }
//                    parsedData << message.name << ",";
//                    parsedData << signal.name << ",";
//                    parsedData << signal.value;
//                    parsedData << "\n";
//                }
//            }
//        }
//    });
//
//
//    thread1.join();
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

    std::string filename = "/apollo/nav/can_client/data/write_and_read_LKAS_message.bin"; // Replace with your filename

    std::ofstream outFile(filename, std::ios::binary);
    // Check if the file is open
    if (!outFile.is_open()) {
        std::cerr << "Error opening file for writing." << std::endl;
        return 1;
    }
    // Write the vector data to the file
    outFile.write(reinterpret_cast<const char*>(sentData.data()), sentData.size());

    // Close the file
    outFile.close();

    // Check for errors during write
    if (!outFile) {
        std::cerr << "Error writing to file." << std::endl;
        return 1;
    }

    // Now open the file, and read it, and parse it to verify it's the same message
    int chunk = 1024;
    std::vector<uint8_t> buffer(chunk);
    std::ifstream file(filename, std::ios::binary);
    file.read(reinterpret_cast<char*>(buffer.data()), chunk);
    size_t bytesRead = file.gcount();
    buffer.resize(bytesRead); // Resize buffer to actual bytes read
    canDevice.receiveMessages(buffer);

    // now get the message  off the  queue
    auto messages = canDevice.getCANMessagesAndClearContainer();
    assert(messages[0].name  == message.name);
    assert(messages[0].address  == message.address);
    assert(messages[0].signals[0].value  == message.signals[0].value);


    return 0;
}
