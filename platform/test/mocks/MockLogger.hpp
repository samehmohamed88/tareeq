#pragma once

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <string>

namespace platform::tests::mocks {
class MockLogger : public ILogger {
public:
    MOCK_METHOD(void, logInfo, (const std::string& message), (override));
    MOCK_METHOD(void, logError, (const std::string& message), (override));
};
}
