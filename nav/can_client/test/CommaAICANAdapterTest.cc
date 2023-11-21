#include "nav/can_client/UsbDevice.h"
#include "nav/can_client/LibUsbDevice.h"
#include "nav/can_client/CommaAICANAdapter.h"
#include "nav/can_client/DeviceInfo.h"

#include <units.h>
#include <libusb-1.0/libusb.h>
#include <gtest/gtest.h>

#include <memory>
#include <utility>

namespace nav {
namespace tests {

static constexpr uint16_t vendorID_ = 0xbbaa;
static constexpr uint16_t productID_ = 0xddcc;
TEST(UsbDeviceTest, TestInitDeviceSucceeds) {
//    auto libUsbDevice = ;
    auto device = std::make_unique<can::UsbDevice<can::LibUsbDevice>>(
            std::make_unique<can::LibUsbDevice>(),
                    vendorID_,
                    productID_,
                    0);


    auto canDevice = can::CommaAICANAdapter<can::UsbDevice<can::LibUsbDevice>>{std::move(device)};
    auto hw = canDevice.getHardwareType();
    EXPECT_EQ(hw, 7);

//    raise std::runtime_error(canDevice.getHardwareType());

//    EXPECT_CALL(*mockLibUsbDevice, libUsbInitDevice())
//    .WillOnce(testing::Return(libusb_error::LIBUSB_SUCCESS));
//    nav::can::UsbDevice<mocks::MockLibUsbDevice> usbDevice{std::move(mockLibUsbDevice), 1, 2};
//    EXPECT_EQ(usbDevice.initDevice(), can::DeviceStatus::SUCCESS);
}

}
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}