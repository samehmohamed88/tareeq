#pragma once

#include "nav/can_client/DeviceInfo.h"

#include <vector>
#include <memory>

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
//    bool setSafetyModel(SafetyModel safety_model, uint16_t safety_param=0U);

    uint8_t getHardwareType();

private:
    std::unique_ptr<Device> device_;
    //    /// the Comma AI Red Panda Vendor ID
//    static constexpr uint16_t vendorID_ = 0xbbaa;
//    /// thee Comma AI Red Panda Product ID
//    static constexpr uint16_t productID_ = 0xddcc;
//    /// interface number taken from https://github.com/commaai/openpilot/blob/master/selfdrive/boardd/panda_comms.cc#L73
//    static constexpr int interfaceNumber_ = 0;
};

//template <class T>
//bool CommaAICANAdapter<T>::setSafetyModel(SafetyModel safetyModel, uint16_t safetyParam) {
//    DeviceStatus status = device_.controlWrite(DeviceRequests::SafetyModel, (uint16_t)safetyModel, safetyParam);
//    return status == DeviceStatus::SUCCESS;
//}

template <class Device>
CommaAICANAdapter<Device>::CommaAICANAdapter(std::unique_ptr<Device> device) :
    device_{std::move(device)}
    {}

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

} // namespace can
} // namespace nav