#include "nav/can_client/UsbDevice.h"
#include "nav/can_client/test/mocks/MockLibUsbDevice.h"

#include <units.h>
#include <libusb-1.0/libusb.h>
#include <gtest/gtest.h>

#include <memory>
#include <utility>

namespace nav {
namespace tests {



/// Tests the device init (which is the libusb init context) succeeds as expected
TEST(UsbDeviceTest, TestInitDeviceSucceeds) {
    auto mockLibUsbDevice = std::make_unique<const mocks::MockLibUsbDevice>();
    EXPECT_CALL(*mockLibUsbDevice, libUsbInitDevice())
            .WillOnce(testing::Return(libusb_error::LIBUSB_SUCCESS));
    nav::can::UsbDevice<mocks::MockLibUsbDevice> usbDevice{std::move(mockLibUsbDevice), 1, 2};
    EXPECT_EQ(usbDevice.initDevice(), can::LibUsbDeviceStatus::SUCCESS);
}

/// Tests if the device init fails (i.e. the libusb context init fails), the device init returns failed to initialize
TEST(UsbDeviceTest, TestInitDeviceFails) {
    auto mockLibUsbDevice = std::make_unique<const mocks::MockLibUsbDevice>();
    EXPECT_CALL(*mockLibUsbDevice, libUsbInitDevice())
            .WillOnce(testing::Return(libusb_error::LIBUSB_ERROR_NO_DEVICE));
    nav::can::UsbDevice<mocks::MockLibUsbDevice> usbDevice{std::move(mockLibUsbDevice), 1, 2};
    EXPECT_EQ(usbDevice.initDevice(), can::LibUsbDeviceStatus::FAILED_TO_INITIALIZE);
}

/// Tests the happy path of device init succeeds and device open succeeds
TEST(UsbDeviceTest, TestOpenDeviceInitSucceeds) {
    auto mockLibUsbDevice = std::make_unique<const mocks::MockLibUsbDevice>();
    EXPECT_CALL(*mockLibUsbDevice, libUsbInitDevice())
            .WillOnce(testing::Return(libusb_error::LIBUSB_SUCCESS));
    nav::can::UsbDevice<mocks::MockLibUsbDevice> usbDevice{std::move(mockLibUsbDevice), 1, 2};
    EXPECT_EQ(usbDevice.openDevice(), can::LibUsbDeviceStatus::SUCCESS);
}

/// Tests when the device init succeeds but the device open fails, the return of openDevice will be a busy device
TEST(UsbDeviceTest, TestOpenDeviceInitSucceedsOpenFails) {
    auto mockLibUsbDevice = std::make_unique<const mocks::MockLibUsbDevice>();
    EXPECT_CALL(*mockLibUsbDevice, libUsbInitDevice())
            .WillOnce(testing::Return(libusb_error::LIBUSB_SUCCESS));
    EXPECT_CALL(*mockLibUsbDevice, libUsbOpenDevice(1, 2))
            .WillOnce(testing::Return(libusb_error::LIBUSB_ERROR_BUSY));
    nav::can::UsbDevice<mocks::MockLibUsbDevice> usbDevice{std::move(mockLibUsbDevice), 1, 2};
    EXPECT_EQ(usbDevice.openDevice(), can::LibUsbDeviceStatus::DEVICE_BUSY);
}

/// Tests if the device init fails that the open device will also fail
TEST(UsbDeviceTest, TestOpenDeviceInitFails) {
    auto mockLibUsbDevice = std::make_unique<const mocks::MockLibUsbDevice>();
    EXPECT_CALL(*mockLibUsbDevice, libUsbInitDevice())
            .WillOnce(testing::Return(libusb_error::LIBUSB_ERROR_IO));
    nav::can::UsbDevice<mocks::MockLibUsbDevice> usbDevice{std::move(mockLibUsbDevice), 1, 2};
    EXPECT_EQ(usbDevice.openDevice(), can::LibUsbDeviceStatus::FAILED_TO_INITIALIZE);
}

/// Tests configure the device succeeds
TEST(UsbDeviceTest, TestConfigureDeviceSuccess) {
    auto mockLibUsbDevice = std::make_unique<const mocks::MockLibUsbDevice>();
    EXPECT_CALL(*mockLibUsbDevice, libUsbInitDevice())
            .WillOnce(testing::Return(libusb_error::LIBUSB_SUCCESS));
    EXPECT_CALL(*mockLibUsbDevice, libUsbOpenDevice(1, 2))
            .WillOnce(testing::Return(libusb_error::LIBUSB_SUCCESS));
    EXPECT_CALL(*mockLibUsbDevice, libUsbSetDefaultConfiguration())
            .WillOnce(testing::Return(libusb_error::LIBUSB_SUCCESS));
    EXPECT_CALL(*mockLibUsbDevice, libUsbDetachKernelDriver(1))
            .WillOnce(testing::Return(libusb_error::LIBUSB_SUCCESS));
    EXPECT_CALL(*mockLibUsbDevice, libUsbClaimInterface(1))
            .WillOnce(testing::Return(libusb_error::LIBUSB_SUCCESS));

    nav::can::UsbDevice<mocks::MockLibUsbDevice> usbDevice{std::move(mockLibUsbDevice), 1, 2};
    EXPECT_EQ(usbDevice.configure(), can::LibUsbDeviceStatus::SUCCESS);
}

/// Tests a configuration write
TEST(UsbDeviceTest, TestWriteAConfigControlWrite) {
    std::vector<uint8_t> data{'d'};
    auto mockLibUsbDevice = std::make_unique<const mocks::MockLibUsbDevice>();
    EXPECT_CALL(*mockLibUsbDevice, libUsbInitDevice())
            .WillOnce(testing::Return(libusb_error::LIBUSB_SUCCESS));
    EXPECT_CALL(*mockLibUsbDevice, libUsbOpenDevice(1, 2))
            .WillOnce(testing::Return(libusb_error::LIBUSB_SUCCESS));
    EXPECT_CALL(*mockLibUsbDevice, libUsbControlTransfer(
            1,
            2,
            3,
            4,
            data.data(),
            data.size(),
            0))
            .WillOnce(testing::Return(libusb_error::LIBUSB_ERROR_IO))
            .WillOnce(testing::Return(libusb_error::LIBUSB_ERROR_BUSY))
            .WillOnce(testing::Return(libusb_error::LIBUSB_SUCCESS));


    nav::can::UsbDevice<mocks::MockLibUsbDevice> usbDevice{std::move(mockLibUsbDevice), 1, 2};
    EXPECT_EQ(usbDevice.controlWrite(1, 2, 3, 4, data,
                                     units::time::millisecond_t{0},
                                     units::time::microsecond_t{500}),
              can::LibUsbDeviceStatus::SUCCESS);
}

/// Tests a configuration write
TEST(UsbDeviceTest, TestReadAConfigControlWrite) {
    std::vector<uint8_t> data{'d'};
    auto mockLibUsbDevice = std::make_unique<const mocks::MockLibUsbDevice>();
    EXPECT_CALL(*mockLibUsbDevice, libUsbInitDevice())
            .WillOnce(testing::Return(libusb_error::LIBUSB_SUCCESS));
    EXPECT_CALL(*mockLibUsbDevice, libUsbOpenDevice(1, 2))
            .WillOnce(testing::Return(libusb_error::LIBUSB_SUCCESS));
    EXPECT_CALL(*mockLibUsbDevice, libUsbControlTransfer(
            1,
            2,
            3,
            4,
            data.data(),
            data.size(),
            0))
            .WillOnce(testing::Return(libusb_error::LIBUSB_ERROR_IO))
            .WillOnce(testing::Return(libusb_error::LIBUSB_ERROR_BUSY))
            .WillOnce(testing::Return(libusb_error::LIBUSB_SUCCESS));


    nav::can::UsbDevice<mocks::MockLibUsbDevice> usbDevice{std::move(mockLibUsbDevice), 1, 2};
    EXPECT_EQ(usbDevice.controlRead(1, 2, 3, 4, data,
                                     units::time::millisecond_t{0},
                                     units::time::microsecond_t{500}),
              can::LibUsbDeviceStatus::SUCCESS);
}

/// Tests write to USB
TEST(UsbDeviceTest, TestWriteToUSB) {

    std::vector<uint8_t> data{'d'};
    std::shared_ptr<int> transferred = std::make_shared<int>(1);
    auto mockLibUsbDevice = std::make_unique<const mocks::MockLibUsbDevice>();
    EXPECT_CALL(*mockLibUsbDevice, libUsbInitDevice())
            .WillOnce(testing::Return(libusb_error::LIBUSB_SUCCESS));
    EXPECT_CALL(*mockLibUsbDevice, libUsbOpenDevice(1, 2))
            .WillOnce(testing::Return(libusb_error::LIBUSB_SUCCESS));
    EXPECT_CALL(*mockLibUsbDevice, libUsbBulkTransfer(
            1,
            data.data(),
            data.size(),
            transferred.get(),
            0))
            .WillOnce(testing::Return(libusb_error::LIBUSB_ERROR_IO))
            .WillOnce(testing::Return(libusb_error::LIBUSB_ERROR_BUSY))
            .WillOnce(testing::Return(libusb_error::LIBUSB_SUCCESS));


    nav::can::UsbDevice<mocks::MockLibUsbDevice> usbDevice{std::move(mockLibUsbDevice), 1, 2};
    EXPECT_EQ(usbDevice.bulkWrite(1, data, transferred,
                                     units::time::millisecond_t{0},
                                     units::time::microsecond_t{500}), can::LibUsbDeviceStatus::SUCCESS);
}

/// Tests read to USB
TEST(UsbDeviceTest, TestReadFromUSB) {

    std::vector<uint8_t> data{'d'};
    std::shared_ptr<int> transferred = std::make_shared<int>(1);
    auto mockLibUsbDevice = std::make_unique<const mocks::MockLibUsbDevice>();
    EXPECT_CALL(*mockLibUsbDevice, libUsbInitDevice())
            .WillOnce(testing::Return(libusb_error::LIBUSB_SUCCESS));
    EXPECT_CALL(*mockLibUsbDevice, libUsbOpenDevice(1, 2))
            .WillOnce(testing::Return(libusb_error::LIBUSB_SUCCESS));
    EXPECT_CALL(*mockLibUsbDevice, libUsbBulkTransfer(
            1,
            data.data(),
            data.size(),
            transferred.get(),
            0))
            .WillOnce(testing::Return(libusb_error::LIBUSB_ERROR_IO))
            .WillOnce(testing::Return(libusb_error::LIBUSB_ERROR_BUSY))
            .WillOnce(testing::Return(libusb_error::LIBUSB_SUCCESS));


    nav::can::UsbDevice<mocks::MockLibUsbDevice> usbDevice{std::move(mockLibUsbDevice), 1, 2};
    EXPECT_EQ(usbDevice.bulkRead(1, data, transferred,
                                  units::time::millisecond_t{0},
                                  units::time::microsecond_t{500}), can::LibUsbDeviceStatus::SUCCESS);
}

} // namespace tests
} // namespace nav


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
