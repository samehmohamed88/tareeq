#include "can_client/LibUsbDevice.h"
//#include "cyber/common/log.h"

#include <libusb-1.0/libusb.h>

namespace nav {
namespace can {

libusb_error LibUsbDevice::libUsbInitDevice()
{
    libusb_context *ctx = nullptr;
    int returnCode = libusb_init(&ctx);

    context_.reset(ctx);

    return static_cast<libusb_error>(returnCode);
}

libusb_error LibUsbDevice::libUsbOpenDevice(uint16_t vendorID, uint16_t productID)  {
    libusb_device_handle *handle = libusb_open_device_with_vid_pid(context_.get(), vendorID, productID);

    if (!handle) {
        return libusb_error::LIBUSB_ERROR_IO;
    }

    deviceHandle_.reset(handle);

    return libusb_error::LIBUSB_SUCCESS;
}

void LibUsbDevice::libUsbReleaseDevice() {
    libusb_release_interface(deviceHandle_.get(), 0);
}

libusb_error LibUsbDevice::libUsbSetDefaultConfiguration() const {
    // Set configuration
    return static_cast<libusb_error>(libusb_set_configuration(deviceHandle_.get(),
                                                              1)); // 1 is typically the default configuration
}

libusb_error LibUsbDevice::libUsbDetachKernelDriver(int interfaceNumber) const {
    int returnCode = 0;
    // Check if kernel driver is active
    if (libusb_kernel_driver_active(deviceHandle_.get(), interfaceNumber) == 1) {
        returnCode = libusb_detach_kernel_driver(deviceHandle_.get(), interfaceNumber);
    }
    return static_cast<libusb_error>(returnCode);
}

libusb_error LibUsbDevice::libUsbClaimInterface(int interfaceNumber) const {
    // Claim interface
    return static_cast<libusb_error>(libusb_claim_interface(deviceHandle_.get(), interfaceNumber));
}

int LibUsbDevice::libUsbControlTransfer(uint8_t bmRequestType,
                                        uint8_t bRequest,
                                        uint16_t wValue,
                                        uint16_t wIndex,
                                        uint8_t *data,
                                        uint16_t wLength,
                                        unsigned int timeout) const {
    return libusb_control_transfer(deviceHandle_.get(), bmRequestType, bRequest, wValue,
                                              wIndex,
                                              data,
                                              wLength, timeout);

}

int LibUsbDevice::libUsbBulkTransfer(uint8_t endpoint,
                                     uint8_t *data,
                                     int length,
                                     int *actual_length,
                                     unsigned int timeout) const {
    return libusb_bulk_transfer(deviceHandle_.get(), endpoint, data, length, actual_length,
                                           timeout);
}

} // namespace can
} //  namespace nav