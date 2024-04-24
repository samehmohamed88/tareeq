#pragma once

#include "units.h"
#include <libusb-1.0/libusb.h>

#include <cstdint>
#include <memory>
#include <vector>

namespace nav {
namespace can {
/// @brief This is a wrapper interface for the libusb C-library.  The intent of this class to bring modern C++ practices
/// to the code and avoid C-Style programming.  It maintains an internal std::unique_ptr for the context,
/// and a std::unique_ptr for the device handle.
/// Each method in this class is a simple direct call to the libusb counter part and does not provide any logic beyond
/// simply calling and returning whatever libusb returns.
/// It is the responsibility of the caller to add logic, error checking and retries.
class LibUsbDeviceInterface {
public:

    /// Calls the `libusb_control_transfer` function
    /// @param bmRequestType the configuration item.
    /// @param bRequest The request field for the setup packet. This is the actual request to be executed.
    /// @param wValue The value field for the setup packet. This is used to pass a parameter to the device, specific to the request.
    /// @param wIndex An offset that is used to provide additional information to the request, commonly 0 but not always.
    /// @param data A pointer to the data buffer. For OUT transfers, this buffer contains the data to send.
    /// For IN transfers, this is where the received data will be stored.
    /// @param wLength The length of the data buffer, in bytes. For OUT transfers, this is the number of bytes to send.
    /// For IN transfers, this is the maximum number of bytes to receive.
    /// @param timeout The time, in milliseconds, that the function should wait before giving up on the transfer.
    /// If the transfer times out, the function will return `LIBUSB_ERROR_TIMEOUT`.
    /// @returns libusb_error a status of either success or timeout, etc.
    virtual int libUsbControlTransfer(uint8_t bmRequestType,
                                               uint8_t bRequest,
                                               uint16_t wValue,
                                               uint16_t wIndex,
                                               uint8_t *data,
                                               uint16_t wLength,
                                               unsigned int timeout) const = 0;

    /// Calls the underlying `libusb_bulk_transfer`
    /// @param endpoint The address of the endpoint where the bulk transfer is to be made.
    /// This includes the direction of the transfer (IN or OUT).
    /// @param data A pointer to the data buffer for the transfer. For OUT transfers, this is the data to send.
    /// For IN transfers, this is where the received data will be stored.
    /// @param length The length of the data buffer, in bytes. This is the number of bytes to be sent for OUT transfers,
    /// or the maximum number of bytes to receive for IN transfers.
    /// @param actual_length A pointer to an integer where the actual length of the data transferred will be stored after,
    /// the transfer completes. This is useful for IN transfers to know how many bytes were actually received.
    /// @param timeout A timeout (in milliseconds) that the function should wait before giving up on the transfer.
    /// If the transfer times out, the function will return `LIBUSB_ERROR_TIMEOUT`.
    /// @returns libusb_error a status of either success or timeout, etc.
    virtual int libUsbBulkTransfer(uint8_t endpoint,
                                            uint8_t *data,
                                            int length,
                                            int *actual_length,
                                            unsigned int timeout) const = 0;

    /// Calls the underlying `libusb_init` to initialize a `libusb_context` that's maintained internally as a std::unique_ptr.
    /// @returns libusb_error a status of either success or timeout, etc.
    virtual libusb_error libUsbInitDevice() = 0;

    /// Calls the underlying `libusb_open_device_with_vid_pid` which initializes the libusb_device_handle that is maintained
    /// internally as a std::unique_ptr.
    /// @param vendorID the vendor ID of the USB device, can be obtained with `dmesg` on most Unix systems.
    /// @param productID the product ID of the USB device, can be obtained with `dmesg` on most Unix systems.
    /// @returns libusb_error a status of either success or timeout, etc.
    virtual libusb_error libUsbOpenDevice(uint16_t vendorID, uint16_t productID) = 0;

    /// Wrapper method for cleaning up resources. It calls `libusb_release_interface`.
    /// @returns libusb_error a status of either success or timeout, etc.
    virtual void libUsbReleaseDevice() = 0;

    /// Calls the underlying `libusb_set_default_configuration`.
    /// @returns libusb_error a status of either success or timeout, etc.
    virtual libusb_error libUsbSetDefaultConfiguration() const = 0;

    /// Calls the underlying `libusb_detach_kernel_driver`.
    /// @param interfaceNumber : The number of the interface for which the kernel driver should be detached.
    /// This is typically the interface you intend to communicate with.
    /// @returns libusb_error a status of either success or timeout, etc.
    virtual libusb_error libUsbDetachKernelDriver(int interfaceNumber) const = 0;

    /// Calls the underlying `libusb_claim_interface`.
    /// @param interfaceNumber : The number of the interface for which the kernel driver should be detached.
    /// This is typically the interface you intend to communicate with.
    /// @returns libusb_error a status of either success or timeout, etc.
    virtual libusb_error libUsbClaimInterface(int interfaceNumber) const = 0;

protected:
    /// a unique pointer wrapping the libusb context internally
    std::unique_ptr<libusb_context, void (*)(libusb_context *)> context_{nullptr, libusb_exit};
    /// a unique pointer for the libusb device handle internally
    std::unique_ptr<libusb_device_handle, void (*)(libusb_device_handle *)> deviceHandle_{nullptr, libusb_close};
};
}
}