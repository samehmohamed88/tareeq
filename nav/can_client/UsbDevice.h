#pragma once

#include "nav/can_client/LibUsbDeviceInterface.h"
#include "nav/can_client/DeviceInfo.h"
#include "cyber/common/log.h"

#include <units.h>
#include <libusb-1.0/libusb.h>

#include <thread>
#include <chrono>
#include <atomic>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

namespace nav {
namespace can {

template <class LibUsbInterface>
class UsbDevice {
public:
    static constexpr uint8_t WriteRequest =
            LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE;
    static constexpr uint8_t ReadRequest =
            LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE;
public:
    /// @param vendorID The USB device Vendor Id.  Can be found with a simple `dmesg` on Unix Systems.
    /// @param productID The USB device Product Id.  Can be found with a simple `dmesg` on Unix Systems.
    /// @param interfaceNumber Typically in LibUsb this value is almost always `1`.
    UsbDevice(std::unique_ptr<LibUsbInterface> libUsbInterface,
              uint16_t vendorID,
              uint16_t productID,
              int interfaceNumber = 1);

    /// Initialize the USB Device Context using libusb context init
    /// @returns DeviceStatus
    DeviceStatus initDevice();
    /// Opens the USB Device using the libusb device handle which accepts a vendor Id and a product Id.
    /// @returns DeviceStatus
    DeviceStatus openDevice();
    /// Sets the default configuration for the USB device using libusb device configuration.
    /// @returns DeviceStatus
    DeviceStatus configure();
    /// Writes a configuration setting to the USB Device
    /// @param bmRequestType the configuration item.
    /// @param bRequest The request field for the setup packet. This is the actual request to be executed.
    /// @param wValue The value field for the setup packet. This is used to pass a parameter to the device, specific to the request.
    /// @param wIndex An offset that is used to provide additional information to the request, commonly 0 but not always.
    /// @param data std::vector character buffer, this is where the received data will be stored.
    /// @param timeout specifies to the libusb the maximum time in milliseconds to wait for the transfer to complete before timing out
    /// Zero value means to return immediately if the transfer cannot be made.
    /// @param sleepDuration number of microseconds to thread sleep in between retry calls to libusb.
    /// @returns DeviceStatus
    DeviceStatus controlRead(const uint8_t bmRequestType,
                                   const uint8_t bRequest,
                                   const uint16_t wValue,
                                   const uint16_t wIndex,
                                   std::vector<uint8_t> &data,
                                   const units::time::millisecond_t timeout = units::time::millisecond_t{0},
                                   const units::time::microsecond_t sleepDuration = units::time::microsecond_t{500});
    /// Reads a configuration setting from the USB Device
    /// @param bmRequestType the configuration item.
    /// @param bRequest The request field for the setup packet. This is the actual request to be executed.
    /// @param wValue The value field for the setup packet. This is used to pass a parameter to the device, specific to the request.
    /// @param wIndex An offset that is used to provide additional information to the request, commonly 0 but not always.
    /// @param data std::vector character buffer. This buffer contains the data to send.
    /// @param timeout specifies to the libusb the maximum time in milliseconds to wait for the transfer to complete before timing out.
    /// Zero value means to return immediately if the transfer cannot be made.
    /// @param sleepDuration number of microseconds to thread sleep in between retry calls to libusb.
    /// @returns DeviceStatus
    DeviceStatus controlWrite(const uint8_t bmRequestType,
                                    const uint8_t bRequest,
                                    const uint16_t wValue,
                                    const uint16_t wIndex,
                                    std::vector<uint8_t> &data,
                                    const units::time::millisecond_t timeout,
                                    const units::time::microsecond_t sleepDuration = units::time::microsecond_t{500});

    /// Reads data from the USB Device.
    /// @param endpoint This specifies the endpoint to communicate with.  It is a combination of address and INPUT.
    /// @param data std::vector character buffer. This buffer contains the data to send.
    /// @param timeout specifies to the libusb the maximum time in milliseconds to wait for the transfer to complete before timing out.
    /// Zero value means to return immediately if the transfer cannot be made.
    /// @param sleepDuration number of microseconds to thread sleep in between retry calls to libusb.
    /// @returns DeviceStatus
    DeviceStatus bulkRead(uint8_t endpoint,
                          std::vector<uint8_t> &data,
                          std::shared_ptr<int> transferred,
                          units::time::millisecond_t timeout = units::time::millisecond_t{0},
                          units::time::microsecond_t sleepDuration = units::time::microsecond_t{500});

    /// Write data to the USB Device.
    /// @param endpoint This specifies the endpoint to communicate with.  It is a combination of address and OUTPUT.
    /// @param data std::vector character buffer. This buffer contains the data to send.
    /// @param timeout specifies to the libusb the maximum time in milliseconds to wait for the transfer to complete before timing out.
    /// Zero value means to return immediately if the transfer cannot be made.
    /// @param sleepDuration number of microseconds to thread sleep in between retry calls to libusb.
    /// @returns DeviceStatus
    DeviceStatus bulkWrite(uint8_t endpoint,
                           std::vector<uint8_t> &data,
                           std::shared_ptr<int> transferred,
                           units::time::millisecond_t timeout = units::time::millisecond_t{0},
                           units::time::microsecond_t sleepDuration = units::time::microsecond_t{500});

    bool attemptReOpenIfConnectionLost();

    bool isCommHealthy() {
        return isCommHealthy_;
    }


private:
    DeviceStatus logConnectionLostAndReturn();
    DeviceStatus logUsbErrorAndReturn(libusb_error returnCode);

    DeviceStatus controlTransferWithRetry(const uint8_t bmRequestType,
                                                const uint8_t bRequest,
                                                const uint16_t wValue,
                                                const uint16_t wIndex,
                                                std::vector<uint8_t> &data,
                                                const units::time::millisecond_t timeout,
                                                const units::time::microsecond_t sleepDuration);

    DeviceStatus bulkTransferWithRetry(const uint8_t endpoint,
                                  std::vector<uint8_t> &data,
                                  std::shared_ptr<int> transferred,
                                  const units::time::millisecond_t timeout,
                                  const units::time::microsecond_t sleepDuration);

    std::unique_ptr<LibUsbInterface> libUsbInterface_;

    uint16_t vendorID_;
    uint16_t productID_;
    int interfaceNumber_;

    bool isKernelDriverDetached_ = false;
    std::string hwSerial_;
    bool isInitialized_ = false;
    bool isOpened_ = false;
    bool isClaimed_ = false;
    std::atomic<bool> isConnected_;
    std::atomic<bool> isCommHealthy_;
    std::mutex mutex_;
};

template <class LibUsbInterface>
UsbDevice<LibUsbInterface>::UsbDevice(std::unique_ptr<LibUsbInterface> libUsbInterface,
                                      uint16_t vendorID,
                                      uint16_t productID,
                                      int interfaceNumber) :
    libUsbInterface_(std::move(libUsbInterface))
    , vendorID_{vendorID}
    , productID_{productID}
    , interfaceNumber_{interfaceNumber}
{}

template <class LibUsbInterface>
DeviceStatus UsbDevice<LibUsbInterface>::logUsbErrorAndReturn(libusb_error returnCode) {
    switch (returnCode) {
        case LIBUSB_ERROR_NOT_FOUND:
            AERROR << "Panda Device not found (Incorrect Interface Number)";
            return DeviceStatus::NO_DEVICE;
        case LIBUSB_ERROR_NO_DEVICE:
            AERROR << "Panda Device not found (it may have been disconnected)";
            return DeviceStatus::NO_DEVICE;
        case LIBUSB_ERROR_BUSY:
            AERROR << "Panda Device returned (Resource busy)";
            return DeviceStatus::DEVICE_BUSY;
        case LIBUSB_ERROR_TIMEOUT:
            AERROR << "USB connection overflow";
            return DeviceStatus::CONNECTION_OVERFLOW;
        case LIBUSB_ERROR_OVERFLOW:
//            AERROR << "Connection With Panda Device Timed Out";
            return DeviceStatus::CONNECTION_OVERFLOW;
        default:
            AERROR << " >>>>>> >>>> >>> "
                   << "Untracked Error Occurred "
                   << returnCode
                   << " >>>>>> >>>> >>> ";
            return DeviceStatus::UNKNOWN_ERROR;
    }
}

template <class LibUsbInterface>
bool UsbDevice<LibUsbInterface>::attemptReOpenIfConnectionLost() {
    if (!isOpened_) {
        auto returnCode = openDevice();
        if (returnCode != DeviceStatus::SUCCESS) {
            return false;
        }
        returnCode = configure();
        if (returnCode != DeviceStatus::SUCCESS) {
            return false;
        }
    }
    return true;
}


template <class LibUsbInterface>
DeviceStatus UsbDevice<LibUsbInterface>::logConnectionLostAndReturn() {
    AERROR << "Connection lost with Panda USB Device";
    return DeviceStatus::NO_DEVICE;
}


template <class LibUsbInterface>
DeviceStatus UsbDevice<LibUsbInterface>::initDevice() {
    auto returnCode = libUsbInterface_->libUsbInitDevice();

    if (returnCode != libusb_error::LIBUSB_SUCCESS) {
        logUsbErrorAndReturn(returnCode);
        return DeviceStatus::FAILED_TO_INITIALIZE;
    }

    isInitialized_ = true;
    return DeviceStatus::SUCCESS;
}


template <class LibUsbInterface>
DeviceStatus UsbDevice<LibUsbInterface>::openDevice() {
    libusb_error returnCode;
    DeviceStatus status;

    if (!isInitialized_) {
        status = initDevice();
        if (status != DeviceStatus::SUCCESS) {
            return status;
        }
    }

    returnCode = libUsbInterface_->libUsbOpenDevice(vendorID_, productID_);
    if (returnCode != libusb_error::LIBUSB_SUCCESS) {
        return logUsbErrorAndReturn(returnCode);
    }

    isOpened_ = true;
    AINFO << "Successfully opened Panda (CAN) USB Device";
    return DeviceStatus::SUCCESS;
}

template <class LibUsbInterface>
DeviceStatus UsbDevice<LibUsbInterface>::configure() {
    libusb_error returnCode;
    if (!attemptReOpenIfConnectionLost()) {
        return logConnectionLostAndReturn();
    }

    returnCode = libUsbInterface_->libUsbDetachKernelDriver(interfaceNumber_);
    if (returnCode != libusb_error::LIBUSB_SUCCESS) {
        isKernelDriverDetached_ = false;
        AERROR << "Could not detach kernel driver: ";
        return logUsbErrorAndReturn(returnCode);
    }
    isKernelDriverDetached_ = true;

    returnCode = libUsbInterface_->libUsbSetDefaultConfiguration();
    if (returnCode != libusb_error::LIBUSB_SUCCESS) {
        return logUsbErrorAndReturn(returnCode);
    }

    returnCode = libUsbInterface_->libUsbClaimInterface(interfaceNumber_);
    if (returnCode != libusb_error::LIBUSB_SUCCESS) {
        isClaimed_ = false;
        AERROR << "Failed to claim interface: ";
        return logUsbErrorAndReturn(returnCode);
    }

    isClaimed_ = true;
    return DeviceStatus::SUCCESS;
}

template <class LibUsbInterface>
DeviceStatus UsbDevice<LibUsbInterface>::controlTransferWithRetry(const uint8_t bmRequestType,
                                                                        const uint8_t bRequest,
                                                                        const uint16_t wValue,
                                                                        const uint16_t wIndex,
                                                                        std::vector<uint8_t> &data,
                                                                        const units::time::millisecond_t timeout,
                                                                        const units::time::microsecond_t sleepDuration) {
    int returnCode ;
    if (!attemptReOpenIfConnectionLost()) {
        return logConnectionLostAndReturn();
    }
    std::lock_guard<std::mutex> lock(mutex_);
    do {
        returnCode = libUsbInterface_->libUsbControlTransfer(
                bmRequestType,
                bRequest,
                wValue,
                wIndex,
                data.data(),
                data.size(),
                static_cast<unsigned int>(timeout.value()));

        if (returnCode != libusb_error::LIBUSB_SUCCESS) {
            if (returnCode == libusb_error::LIBUSB_ERROR_NO_DEVICE) {
                isConnected_ = false;
                return logConnectionLostAndReturn();
            }
        }
        if (returnCode < 0) {
            // Introduce a delay before retrying
            // chrono microseconds excepts an int value and the units time millisecond is a double value, so we cast
            std::this_thread::sleep_for(std::chrono::microseconds(int(sleepDuration.value())));
            AERROR << "Control Write Error "
                    << libusb_strerror((enum libusb_error)returnCode)
                   << " . Retrying ...";
        }
        // TODO: check other errors, or is simply retrying okay?
    } while (returnCode < 0 );
    return DeviceStatus::SUCCESS;
}


template <class LibUsbInterface>
DeviceStatus UsbDevice<LibUsbInterface>::controlWrite(const uint8_t bmRequestType,
                                                            const uint8_t bRequest,
                                                            const uint16_t wValue,
                                                            const uint16_t wIndex,
                                                            std::vector<uint8_t> &data,
                                                            const units::time::millisecond_t timeout,
                                                            const units::time::microsecond_t sleepDuration) {
    return controlTransferWithRetry(bmRequestType,
                                    bRequest,
                                    wValue,
                                    wIndex,
                                    data,
                                    timeout,
                                    sleepDuration);
}


template <class LibUsbInterface>
DeviceStatus UsbDevice<LibUsbInterface>::controlRead(const uint8_t bmRequestType,
                                                           const uint8_t bRequest,
                                                           const uint16_t wValue,
                                                           const uint16_t wIndex,
                                                           std::vector<uint8_t> &data,
                                                           const units::time::millisecond_t timeout,
                                                           const units::time::microsecond_t sleepDuration) {
    return controlTransferWithRetry(bmRequestType,
                                    bRequest,
                                    wValue,
                                    wIndex,
                                    data,
                                    timeout,
                                    sleepDuration);
}


template <class LibUsbInterface>
DeviceStatus UsbDevice<LibUsbInterface>::bulkTransferWithRetry(const uint8_t endpoint,
                                                          std::vector<uint8_t> &data,
                                                          std::shared_ptr<int> transferred,
                                                          const units::time::millisecond_t timeout,
                                                          const units::time::microsecond_t sleepDuration) {
    int returnCode ;
    if (!attemptReOpenIfConnectionLost()) {
        // we can only return a uint8_t since
        // it must represent the actual number of bytes transferred
        // so we return 0 and not any negative value
        logConnectionLostAndReturn();
        return DeviceStatus::SUCCESS;
    }
    std::lock_guard<std::mutex> lock(mutex_);

    do {
        returnCode = libUsbInterface_->libUsbBulkTransfer(
                endpoint,
                data.data(),
                data.size(),
                transferred.get(),
                static_cast<unsigned int>(timeout.value()));

        if (static_cast<libusb_error>(returnCode) == libusb_error::LIBUSB_SUCCESS) {
//            AINFO << "Successfully read "
//                  << std::to_string(*(transferred.get()))
//                  << " bytes from Device";
            return DeviceStatus::SUCCESS;
        }
        DeviceStatus status = logUsbErrorAndReturn(static_cast<libusb_error>(returnCode));
        switch (status) {
            case DeviceStatus::CONNECTION_OVERFLOW:
                // we try again
                isCommHealthy_ = false;
                break;
            case DeviceStatus::CONNECTION_TIMEOUT:
                // timeout is okay to exit, the recv still happened
                return DeviceStatus::SUCCESS;
            case DeviceStatus::NO_DEVICE:
                return DeviceStatus::NO_DEVICE;
            default:
                // safe to try again
                AINFO << "Retrying bulk transfer due to "
                       << libusb_strerror((enum libusb_error)returnCode);
        }
        // Introduce a delay before retrying
        // chrono microseconds excepts an int value and the units time millisecond is a double value, so we cast
        std::this_thread::sleep_for(std::chrono::microseconds(int(sleepDuration.value())));
    } while (returnCode != 0);

    return DeviceStatus::SUCCESS;
}
template <class LibUsbInterface>
DeviceStatus UsbDevice<LibUsbInterface>::bulkRead(const uint8_t endpoint,
                                             std::vector<uint8_t> &data,
                                             std::shared_ptr<int> transferred,
                                             const units::time::millisecond_t timeout,
                                             const units::time::microsecond_t sleepDuration) {
    return bulkTransferWithRetry(endpoint,
                                 data,
                                 transferred,
                                 timeout,
                                 sleepDuration);

}
template <class LibUsbInterface>
DeviceStatus UsbDevice<LibUsbInterface>::bulkWrite(const uint8_t endpoint,
                                              std::vector<uint8_t> &data,
                                              std::shared_ptr<int> transferred,
                                              const units::time::millisecond_t timeout,
                                              const units::time::microsecond_t sleepDuration) {
    return bulkTransferWithRetry(endpoint,
                                 data,
                                 transferred,
                                 timeout,
                                 sleepDuration);

}

}
}