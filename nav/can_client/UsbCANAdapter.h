#pragma once

#include "LibUsbDeviceInterface.h"

#include "units.h"


#include <cstdint>
#include <mutex>
#include <string>
#include <vector>
#include <memory>

namespace l3p {
    namespace can {

        template<class USBDevice>
        class UsbCANAdapter {
        public:
            UsbCANAdapter();

            ~UsbCANAdapter();

            /// Writes a configuration setting to the Comma AI CAN Device
            /// @param request the configuration item
            /// @param value the value to set for the configuration item
            /// @param index an offset that is used to provide additional information to the request, commonly 0 but not always
            /// @param timeout specifies to the libusb the maximum time in milliseconds to wait for the transfer to complete before timing out.
            /// Zero value means to return immediately if the transfer cannot be made
            int controlWrite(const uint8_t request, const uint16_t value, const uint16_t index,
                             const units::time::millisecond_t timeout = units::time::millisecond_t{0});

            int controlRead(uint8_t request, uint16_t param1, uint16_t param2, unsigned char *data, uint16_t length,
                            const units::time::millisecond_t timeout = units::time::millisecond_t{0});

            int bulkWrite(unsigned char endpoint, unsigned char *data, int length,
                          const units::time::millisecond_t timeout = units::time::millisecond_t{0});

            int bulkRead(unsigned char endpoint, unsigned char *data, int length,
                         const units::time::millisecond_t timeout = units::time::millisecond_t{0});

            void cleanup();

            bool isConnected() const;

            bool isCommHealthy() const;

        private:


            USBDevice usbDevice_;
            std::mutex mutex_;

            void
            processReturnCode(int return_code, std::string function_name, units::time::millisecond_t sleep_duration);

            int libusbControlTransferWithRetry(
                    const uint8_t bmRequestType,
                    const uint8_t bRequest,
                    const uint16_t wValue,
                    const uint16_t wIndex,
                    unsigned char *data,
                    const uint16_t wLength,
                    const units::time::millisecond_t timeout,
                    const units::time::microsecond_t sleep_duration = units::time::microsecond_t{500});

            int libusbBulkTransferWithRetry(
                    unsigned char endpoint, unsigned char *data, int length, const units::time::millisecond_t timeout,
                    const units::time::microsecond_t sleep_duration = units::time::microsecond_t{500});
        };

        template <class USBDevice>
        UsbCANAdapter<USBDevice>::UsbCANAdapter() :
                is_connected_{false}, is_comm_healthy_{false}, context_{std::move(context)},
                device_handle_{std::move(device_handle)} {

            libusb_device_handle *handle = libusb_open_device_with_vid_pid(context_.get(), vendor_id_, product_id_);
            if (!handle) {
                AERROR << "Panda (CAN) USB Device not found";
            }
            device_handle_.reset(handle);

            // Set configuration
            int config_result = libusb_set_configuration(device_handle_.get(),
                                                         1); // 1 is typically the default configuration
            if (config_result != LIBUSB_SUCCESS) {
                AERROR
                        << "Failed to set device configuration: "
                        << std::string(libusb_error_name(config_result));
            }

            // Check if kernel driver is active
            if (libusb_kernel_driver_active(device_handle_.get(), interface_number_) == 1) {
                int detach_result = libusb_detach_kernel_driver(device_handle_.get(), interface_number_);
                if (detach_result != LIBUSB_SUCCESS) {
                    AERROR << "Could not detach kernel driver: "
                           << std::string(libusb_error_name(detach_result));
                }
                kernel_driver_detached_ = true;
            }

            // Claim interface
            int claim_result = libusb_claim_interface(device_handle_.get(), interface_number_);
            if (claim_result != LIBUSB_SUCCESS) {
                AERROR << "Failed to claim interface: "
                       << std::string(libusb_error_name(claim_result));
            }
            AINFO << "Successfully connected to Panda (CAN) USB Device";
            is_connected_ = true;
        }

        template <class USBDevice>
        UsbCANAdapter<USBDevice>::~UsbCANAdapter() {
            std::lock_guard<std::mutex> lock{mutex_};
            cleanup();
            is_connected_ = false;

            // Release the claimed interface
            if (device_handle_ != nullptr) {
                libusb_release_interface(device_handle_.get(), interface_number_);
            }

            // Reattach the kernel driver if it was detached
            if (kernel_driver_detached_) {
                libusb_attach_kernel_driver(device_handle_.get(), interface_number_);
            }
        }

        template <class USBDevice>
        void UsbCANAdapter<USBDevice>::cleanup() {
            if (device_handle_) {
                libusb_release_interface(device_handle_.get(), 0);
                libusb_close(device_handle_.get());
            }

            if (context_) {
                libusb_exit(context_.get());
            }
        }

        template <class USBDevice>
        void UsbCANAdapter<USBDevice>::processReturnCode(int return_code, std::string function_name,
                                              const units::time::millisecond_t sleep_duration) {

            switch (return_code) {
                case LIBUSB_ERROR_NO_DEVICE: {
                    AERROR << " Error cause is : lost connection";
                    is_connected_ = false;
                    break;
                }
                case LIBUSB_ERROR_OVERFLOW: {
                    AERROR << " Error cause is : overflow";
                    is_comm_healthy_ = false;
                }
                default: {
                    // Log the error and retry
                    AERROR << "libusb_control_transfer failed: "
                           << libusb_error_name(return_code)
                           << ", retrying...";
                    // Introduce a delay before retrying
                    // chrono microseconds excepts an int value and the units time millisecond is a double value, so we cast
                    std::this_thread::sleep_for(std::chrono::microseconds(int(sleep_duration.value())));
                    break;
                }
            }
        }

        template <class USBDevice>
        int UsbCANAdapter<USBDevice>::libusbControlTransferWithRetry(
                const uint8_t bmRequestType,
                const uint8_t bRequest,
                const uint16_t wValue,
                const uint16_t wIndex,
                const uint16_t wLength,
                const units::time::millisecond_t timeout,
                const units::time::microsecond_t sleep_duration) {
            int return_code;
            std::lock_guard<std::mutex> lock{mutex_};
            while (true) {
                return_code = libusb_control_transfer(device_handle_.get(), bmRequestType, bRequest, wValue, wIndex,
                                                      NULL,
                                                      wLength, timeout.value());
                processReturnCode(return_code, __func__, sleep_duration);
                switch (return_code) {
                    // Break out of the loop in case of success, specific errors or conditions
                    case LIBUSB_SUCCESS || LIBUSB_ERROR_NO_DEVICE || LIBUSB_ERROR_TIMEOUT:
                        return return_code;
                }
            }
        }

        template <class USBDevice>
        int UsbCANAdapter<USBDevice>::libusbBulkTransferWithRetry(unsigned char endpoint, unsigned char *data, int length,
                                                       const units::time::millisecond_t timeout,
                                                       units::time::microsecond_t sleep_duration) {
            int return_code;
            int transferred = 0;
            std::lock_guard<std::mutex> lock{mutex_};
            while (true) {
                // Try sending can messages. If the receive buffer on the can_client panda is full, it will NAK
                // and libusb will try again. After 5ms, it will time out. We will drop the messages.
                return_code = libusb_bulk_transfer(device_handle_.get(), endpoint, data, length, &transferred,
                                                   timeout.value());
                processReturnCode(return_code, __func__, sleep_duration);
                switch (return_code) {
                    case LIBUSB_SUCCESS:
                        return transferred;
                }
            }
            return transferred; // Return the last error code
        }

        template <class USBDevice>
        int UsbCANAdapter<USBDevice>::controlWrite(const uint8_t request, const uint16_t value, const uint16_t index,
                                        units::time::millisecond_t timeout) {
            return libusbControlTransferWithRetry(bm_request_write_,
                                                  request,
                                                  value,
                                                  index,
                                                  NULL,
                                                  0,
                                                  timeout);
        }

        template <class USBDevice>
        int UsbCANAdapter<USBDevice>::controlRead(const uint8_t bRequest, const uint16_t wValue, uint16_t wIndex, unsigned char *data,
                                   const uint16_t wLength, units::time::millisecond_t timeout) {
            return libusbControlTransferWithRetry(bm_request_read_,
                                                  bRequest,
                                                  wValue,
                                                  wIndex,
                                                  data,
                                                  wLength,
                                                  timeout);
        }

        template <class USBDevice>
        int UsbCANAdapter<USBDevice>::bulkWrite(unsigned char endpoint, unsigned char *data, int length,
                                     units::time::millisecond_t timeout) {
            return libusbBulkTransferWithRetry(
                    endpoint, data, length, timeout);
        }

        template <class USBDevice>
        int UsbCANAdapter<USBDevice>::bulkRead(unsigned char endpoint, unsigned char *data, int length,
                                    units::time::millisecond_t timeout) {
            return libusbBulkTransferWithRetry(
                    endpoint, data, length, timeout);
        }

        template <class USBDevice>
        bool UsbCANAdapter<USBDevice>::isConnected() const {
            return is_connected_;
        }

        template <class USBDevice>
        bool UsbCANAdapter<USBDevice>::isCommHealthy() const {
            return is_comm_healthy_;
        }

    } // namespace can
} // namespace nav