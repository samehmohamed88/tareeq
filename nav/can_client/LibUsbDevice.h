#pragma once

#include "nav/can_client/LibUsbDeviceInterface.h"

#include "units.h"
#include <libusb-1.0/libusb.h>

#include <atomic>
#include <memory>

namespace nav {
namespace can {

/// @brief Implementation of the libusb Device Interface.
class LibUsbDevice : public LibUsbDeviceInterface {
public:
    LibUsbDevice();

    int libUsbControlTransfer(const uint8_t bmRequestType,
                                      const uint8_t bRequest,
                                      const uint16_t wValue,
                                      const uint16_t wIndex,
                                      unsigned char *data,
                                      const uint16_t wLength,
                                      const units::time::millisecond_t timeout) override;
    int libUsbBulkTransfer(unsigned char endpoint, unsigned char *data, int length,
                           std::shared_ptr<int> transferred,
                           const units::time::millisecond_t timeout) override;
    int libUsbInitDevice() override;
    int libUsbOpenDevice(uint16_t vendorID_, uint16_t productID_) override;
    void libUsbReleaseDevice() override;
    int libUsbSetDefaultConfiguration() override;
    int libUsbDetachKernelDriver(int interfaceNumber) override;
    int libUsbClaimInterface(int interfaceNumber) override;

};
}
}
