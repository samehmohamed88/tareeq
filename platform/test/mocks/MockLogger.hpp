#pragma once

#include "platform/logging/LoggerFactory.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <string>

namespace platform::tests::mocks {

class MockLogger : public devices::ILogger
{
public:
    MOCK_METHOD(void, logInfo, (const std::string& message), (override));

    MOCK_METHOD(void, logError, (const std::string& message), (override));
};
} // namespace platform::tests::mocks
