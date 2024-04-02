
#include "platform/devices/DeviceManager.hpp"
#include "platform/test/mocks/MockISerialPort.hpp"
#include "platform/test/mocks/MockLogger.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>

using namespace platform::devices;
using namespace platform::tests::mocks;
using ::testing::_;
using ::testing::Invoke;
using ::testing::Return;

class SerialDeviceTest : public ::testing::Test
{
protected:
    std::shared_ptr<MockLogger> mockLogger_;
    std::shared_ptr<MockSerialPort> mockSerialPort_;
    std::shared_ptr<DeviceManager<MockSerialPort, MockLogger>> serialDeviceManager_;

    void SetUp() override
    {
        mockSerialPort_ = std::make_shared<MockSerialPort>();
        mockLogger_ = std::make_shared<MockLogger>();
        serialDeviceManager_ =
            std::make_shared<DeviceManager<MockSerialPort, MockLogger>>(mockSerialPort_, mockLogger_);
    }
};

TEST_F(SerialDeviceTest, WriteSuccess)
{
    const std::string test_data = "Hello, World!";

    EXPECT_CALL(*mockSerialPort_, write(_)).Times(1).WillOnce(Return());

    EXPECT_CALL(*mockLogger_, logError(_)).Times(0); // No error should be logged

    ASSERT_NO_THROW(serialDeviceManager_->write(test_data));
}
