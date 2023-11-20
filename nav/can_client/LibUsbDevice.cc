#include "nav/can_client/LibUsbDevice.h"
//#include "cyber/common/log.h"

#include <libusb-1.0/libusb.h>

namespace nav {
namespace can {
LibUsbDevice::LibUsbDevice() :
    context_{nullptr, libusb_exit}
    , deviceHandle_{nullptr, libusb_close}
{}

int LibUsbDevice::libUsbInitDevice()
{
    libusb_context *ctx = nullptr;
    int returnCode = libusb_init(&ctx);
    context_.reset(ctx);
    return returnCode;
}

int LibUsbDevice::libUsbOpenDevice(uint16_t vendorID, uint16_t productID)  {
    libusb_device_handle *handle = libusb_open_device_with_vid_pid(context_.get(), vendorID, productID);
    if (!handle) {
        return -1;
    }
    deviceHandle_.reset(handle);
    return 0;
}

void LibUsbDevice::libUsbReleaseDevice() {
    libusb_release_interface(deviceHandle_.get(), 0);
}

int LibUsbDevice::libUsbSetDefaultConfiguration() {
    // Set configuration
    return libusb_set_configuration(deviceHandle_.get(),
                                              1); // 1 is typically the default configuration
}

int LibUsbDevice::libUsbDetachKernelDriver(int interfaceNumber) {
    int returnCode = 0;
    // Check if kernel driver is active
    if (libusb_kernel_driver_active(deviceHandle_.get(), interfaceNumber) == 1) {
        returnCode = libusb_detach_kernel_driver(deviceHandle_.get(), interfaceNumber);
    }
    return returnCode;
}

int LibUsbDevice::libUsbClaimInterface(int interfaceNumber) {
    // Claim interface
    return libusb_claim_interface(deviceHandle_.get(), interfaceNumber);
}

int LibUsbDevice::libUsbControlTransfer(const uint8_t bmRequestType,
        const uint8_t bRequest,
        const uint16_t wValue,
        const uint16_t wIndex,
        unsigned char *data,
        const uint16_t wLength,
        const units::time::millisecond_t timeout) {
    return libusb_control_transfer(deviceHandle_.get(), bmRequestType, bRequest, wValue, wIndex,
                                   data,
                                   wLength, timeout.value());
}

int LibUsbDevice::libUsbBulkTransfer(unsigned char endpoint, unsigned char *data, int length,
                                     std::shared_ptr<int> transferred,
                                     const units::time::millisecond_t timeout) {
    return libusb_bulk_transfer(deviceHandle_.get(), endpoint, data, length, transferred.get(),
                                timeout.value());
}

} // namespace can
} //  namespace nav