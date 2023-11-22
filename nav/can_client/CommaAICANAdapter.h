#pragma once

#include "nav/can_client/DeviceInfo.h"

#include <vector>
#include <memory>
#include <fstream>
#include <iterator>
#include <string>

namespace nav {
namespace can {

template <class Device>
class CommaAICANAdapter {
public:
    CommaAICANAdapter(std::unique_ptr<Device> device);

    /// Sets the safety model on the CAN Device
    /// The Comma AI CAN Device has a specific safety model where it starts in silent mode with No Output to prevent
    /// accidental movements/steering
    /// taken from https://github.com/commaai/cereal/blob/416c3d531c90ce16498d782bf383625a857ee74c/car.capnp#L567C8-L567C19
    bool setSafetyModel(SafetyModel safety_model, uint16_t safety_param=0U);

    uint8_t getHardwareType();
    bool getCANMessages();

private:
    std::unique_ptr<Device> device_;
    std::ofstream output_file_;
    //    /// the Comma AI Red Panda Vendor ID
//    static constexpr uint16_t vendorID_ = 0xbbaa;
//    /// thee Comma AI Red Panda Product ID
//    static constexpr uint16_t productID_ = 0xddcc;
//    /// interface number taken from https://github.com/commaai/openpilot/blob/master/selfdrive/boardd/panda_comms.cc#L73
//    static constexpr int interfaceNumber_ = 0;
};

template <class T>
bool CommaAICANAdapter<T>::setSafetyModel(SafetyModel safetyModel, uint16_t safetyParam) {
    DeviceStatus status = device_.controlWrite(DeviceRequests::SafetyModel, static_cast<uint16_t>(safetyModel), safetyParam);
    return status == DeviceStatus::SUCCESS;
}

template <class Device>
CommaAICANAdapter<Device>::CommaAICANAdapter(std::unique_ptr<Device> device) :
    device_{std::move(device)}
    , output_file_{"/apollo/can_str.txt"}
    {
//        output_file_ = std::ofstream<uint8_t>("./example.txt");
    }

template <class Device>
uint8_t CommaAICANAdapter<Device>::getHardwareType() {
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

struct __attribute__((packed)) CANHeader {
    uint8_t reserved : 1;
    uint8_t bus : 3;
    uint8_t data_len_code : 4;
    uint8_t rejected : 1;
    uint8_t returned : 1;
    uint8_t extended : 1;
    uint32_t addr : 29;
    uint8_t checksum : 8;
};

struct CANFrame {
    long address;
    std::string dat;
    long busTime;
    long src;
};

template <class Device>
bool CommaAICANAdapter<Device>::getCANMessages() {
    // holds the value of the actual number of bytes read

    constexpr auto vectorSize = 0x4000U;
    std::shared_ptr<int> transferred = std::make_shared<int>(0);
    std::vector<uint8_t> data;
    data.reserve(vectorSize);
    for (unsigned int i =0; i < vectorSize; i++) {
        data.push_back(0);
    }
    std::cout << "We started with " << std::to_string(data[0]) << std::endl;
    DeviceStatus status = device_->bulkRead(
            static_cast<uint8_t>(DeviceRequests::READ_CAN_BUS),
            data,
            transferred);
    std::cout << "Panda returned " << std::to_string(data[0]) << std::endl;
//    std::ofstream oufile("/home/sameh/can_example.txt"+std::to_string(*transferred));
//    oufile.open();
    std::ostream_iterator<std::string> output_iterator(output_file_, "\n");
    std::copy(std::begin(data), std::end(data), output_iterator);
//    for (int i = 0; i  < *transferred; i++) {
//        oufile << data[i];
//    }
//    oufile << "\n";
    if (status == DeviceStatus::SUCCESS) {
        return true;
    }
    return false;
}

} // namespace can
} // namespace nav