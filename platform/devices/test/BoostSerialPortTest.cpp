
#include "platform/devices/BoostSerialPort.hpp"  // Adjust the include path to match your project structure
#include "platform/test/mocks/MockIAsioOperations.hpp"
#include "platform/test/mocks/MockLogger.hpp"


#include <gtest/gtest.h>
#include <gmock/gmock.h>


using namespace platform::devices;
using ::testing::_;
using ::testing::Invoke;
using ::testing::Return;

class BoostSerialPortTest : public ::testing::Test {
protected:
    std::shared_ptr<MockAsioOperations> mockAsioOperations;
    std::shared_ptr<MockLogger> mockLogger;
    std::unique_ptr<BoostSerialPort<MockAsioOperations, MockLogger>> port;

    void SetUp() override {
        mockAsioOperations = std::make_shared<MockAsioOperations>();
        mockLogger = std::make_shared<MockLogger>();
        port = std::make_unique<BoostSerialPort<MockAsioOperations, MockLogger>>(mockAsioOperations, mockLogger, "COM3");
    }
};


TEST_F(BoostSerialPortTest, WriteSuccess) {
    const std::string test_data = "Hello, World!";

    EXPECT_CALL(*mockAsioOperations, write(_, _))
        .Times(1)
        .WillOnce(Invoke([&](auto&, auto& buffer) {
            // Simulate successful write operation
        }));

    EXPECT_CALL(*mockLogger, logError(_))
        .Times(0);  // No error should be logged

    ASSERT_NO_THROW(port->write(test_data));
}


TEST_F(BoostSerialPortTest, WriteError) {
    const std::string test_data = "Hello, World!";

    EXPECT_CALL(*mockAsioOperations, write(_, _))
        .Times(1)
        .WillOnce(Invoke([&](auto&, auto&) {
            throw boost::system::system_error(boost::asio::error::write_failed);
        }));

    EXPECT_CALL(*mockLogger, logError(_))
        .Times(2);  // Expect two error logs: one for the error and another for the exception message

    ASSERT_THROW(port->write(test_data), boost::system::system_error);
}

