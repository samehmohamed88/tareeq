
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

bool IN_CAR = true;

std::string getFileNameAsCurrentTimestamp() {
    auto currentTime = std::chrono::system_clock::now();
    std::time_t time = std::chrono::system_clock::to_time_t(currentTime);
    std::tm timeinfo = *std::localtime(&time);
    char buffer[80];  // Adjust the buffer size as needed
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d-%H-%M-%S", &timeinfo);
    std::string filename = std::string("can-messages-received_") + std::string(buffer) + ".txt";  // You can change the file extension as needed
    return filename;
}

std::thread CreateWriteFileThread(can::CommaAICANInterfaceWithBoostBuffer<can::UsbDevice<can::LibUsbDevice>>& canDevice) {
    auto thread1 = std::thread([&]{
        std::ofstream parsedData(getFileNameAsCurrentTimestamp());
        while (!EXIT) {
            auto messages = canDevice.getCANMessagesAndClearContainer();
            for (auto const& message : messages) {
                parsedData << message.name << ",";
                parsedData << message.name << ",";
                parsedData << signal.value;
                parsedData << "\n";
            }
        }
    });
}

std::thread CreateReceiveThread(can::CommaAICANInterfaceWithBoostBuffer<can::UsbDevice<can::LibUsbDevice>>& canDevice) {
    std::vector<uint8_t> chunck{};
    auto thread = std::thread([&]{
        while (!EXIT) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            canDevice.receiveMessages(chunck, IN_CAR);
        }
    });
    return thread;
}


int main() {
//    using namespace nav;
//    static constexpr uint16_t vendorID_ = 0xbbaa;
//    static constexpr uint16_t productID_ = 0xddcc;
//
//    std::unordered_map<std::string , int32_t > messageMap{};
//
//    auto device = std::make_unique<can::UsbDevice<can::LibUsbDevice>>(
//            std::make_unique<can::LibUsbDevice>(),
//                    vendorID_,
//                    productID_,
//                    0);
//
//    auto canDevice = can::CommaAICANInterfaceWithBoostBuffer<can::UsbDevice<can::LibUsbDevice>>{std::move(device)};
//    auto hw = canDevice.getHardwareType();
//    assert(hw == 7);
//    std::cout << "Hardware Type == " << std::to_string(hw) << std::endl;
//
//    std::string filename = "/apollo/nav/can_client/data/can_output_2023-11-25_22-32-26.bin"; // Replace with your filename
//    std::ifstream file(filename, std::ios::binary);
//    const size_t chunkSize = 1024; // Size of each chunk in bytes
//    std::vector<uint8_t> buffer(chunkSize);
//
//    std::ofstream parsedData("/apollo/nav/can_client/data/can_parsed_2023-11-27_09-32-26.csv");
//
//
//    auto thread1 = std::thread([&]{
//        while (file) {
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
//    auto thread2 = std::thread([&]{
//        while (file) {
////            std::this_thread::sleep_for(std::chrono::milliseconds(75));
//            file.read(reinterpret_cast<char*>(buffer.data()), chunkSize);
//            size_t bytesRead = file.gcount();
////            std::cout << " number of bytes read = " << bytesRead << std::endl;
//            if (bytesRead > 0) {
//                buffer.resize(bytesRead); // Resize buffer to actual bytes read
//                canDevice.receiveMessages(buffer, false);
//                buffer.resize(chunkSize); // Resize back for next read
//            }
//        }
//    });
//    thread1.join();
//    thread2.join();
//    int total = 0;
//    std::cout << "All clear.  Found the following" << std::endl;
//    for (const auto& kv : messageMap) {
//        std::cout << "Key: " << kv.first << ", Value: " << kv.second << std::endl;
//        total += kv.second;
//    }
//    std::cout << "For a total of " << std::to_string(total) << " messages" << std::endl;
//    return 0;
}
