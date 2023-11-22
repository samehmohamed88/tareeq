
#include "nav/can_client/UsbDevice.h"
#include "nav/can_client/LibUsbDevice.h"
#include "nav/can_client/CommaAICANAdapter.h"

#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <libusb-1.0/libusb.h>

#include <iostream>

using namespace nav;

void perform_usb_transfer(const boost::system::error_code& /*e*/, boost::asio::steady_timer* t, int* count, can::CommaAICANAdapter<can::UsbDevice<can::LibUsbDevice>>* canDevice) {
    if (*count < 100) {
//        std::cout << "USB Transfer " << *count << std::endl;
        ++(*count);

        // Example USB bulk transfer parameters
//        unsigned char endpoint = 0x81; // Example endpoint
//        unsigned char data[512];       // Buffer for data
//        int actual_length;             // Will store the actual length of data received
//        int timeout = 1000;            // Timeout in ms
//
//        // Perform the bulk transfer
//        int transfer_result = libusb_bulk_transfer(handle, endpoint, data, sizeof(data), &actual_length, timeout);
//
//        if (transfer_result == 0) {
//            // Handle successful transfer
//        } else {
//            // Handle errors
//        }

        (*canDevice).getCANMessages();
        // Schedule the next transfer
        t->expires_at(t->expiry() + boost::asio::chrono::milliseconds(10));
        t->async_wait(boost::bind(perform_usb_transfer, boost::asio::placeholders::error, t, count, canDevice));
    }
}

int main() {
    using namespace nav;
    static constexpr uint16_t vendorID_ = 0xbbaa;
    static constexpr uint16_t productID_ = 0xddcc;

    boost::asio::io_context io;

    auto device = std::make_unique<can::UsbDevice<can::LibUsbDevice>>(
            std::make_unique<can::LibUsbDevice>(),
                    vendorID_,
                    productID_,
                    0);


    auto canDevice = can::CommaAICANAdapter<can::UsbDevice<can::LibUsbDevice>>{std::move(device)};
    auto hw = canDevice.getHardwareType();
    assert(hw == 7);
    std::cout << "Hardware Type == " << std::to_string(hw) << std::endl;

    int count = 0;
    boost::asio::steady_timer t(io, boost::asio::chrono::milliseconds(10));
    t.async_wait(boost::bind(perform_usb_transfer, boost::asio::placeholders::error, &t, &count, &canDevice));

    io.run();

    // Cleanup: Close the USB device and deinitialize libusb
//    libusb_close(usb_handle);
//    libusb_exit(NULL);

    return 0;
}
