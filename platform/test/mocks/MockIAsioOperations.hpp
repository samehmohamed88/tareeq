#pragma once

#include "platform/io/IAsioOperations.hpp"

#include <boost/asio.hpp>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <string>

namespace platform::tests::mocks {
    class MockAsioOperations : public io::IAsioOperations {
    public:
        MOCK_METHOD(void, write,(boost::asio::serial_port &serial, const boost::asio::const_buffer &buffer), (override)

        );
        MOCK_METHOD(std::size_t, read, (boost::asio::serial_port &serial,
        boost::asio::mutable_buffer buffer
        ), (override));
    };
}
