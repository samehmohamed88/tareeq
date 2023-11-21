#pragma once

#include <libusb-1.0/libusb.h>

namespace nav {
namespace can {

enum class DeviceStatus {
    SUCCESS = libusb_error::LIBUSB_SUCCESS,
    FAILED_TO_INITIALIZE = libusb_error::LIBUSB_ERROR_IO,
    NO_DEVICE = LIBUSB_ERROR_NO_DEVICE,
    DEVICE_BUSY = LIBUSB_ERROR_BUSY,
    CONNECTION_TIMEOUT = LIBUSB_ERROR_TIMEOUT,
    CONNECTION_OVERFLOW = LIBUSB_ERROR_OVERFLOW,
    UNKNOWN_ERROR = -100

};

/// The Comma AI CAN Device has a specific safety model where it starts in silent mode with No Output to prevent
/// accidental movements/steering
/// taken from https://github.com/commaai/cereal/blob/416c3d531c90ce16498d782bf383625a857ee74c/car.capnp#L567C8-L567C19
enum class SafetyModel {
    NoOutput = 0,
    AllOutput = 17
};

/// @brief Enumerations of the address of the request to the Comma AI CAN Device for reading and writing CAN frames and device
/// configurations
enum class DeviceRequests {
    CommaAIDeviceName = 0xc1,
    ResetCommunications = 0xc0,
    SafetyModel = 0xdc,
    HardwareType = 0xc1,
    READ_CAN_BUS = 0x81
};

} // namespace can
} // namespace nav