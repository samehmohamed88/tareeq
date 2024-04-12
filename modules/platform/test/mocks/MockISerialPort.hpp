#pragma once

#include "platform/devices/ISerialPort.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <string>

namespace platform::tests::mocks {
class MockSerialPort : public devices::ISerialPort
{
public:
    using ReadCallback = std::function<void(const std::string&)>;
    MOCK_METHOD(void, write, (const std::string& data), (override));

    MOCK_METHOD(void, read, (const ReadCallback& callback), (override));
};

} // namespace platform::tests::mocks
