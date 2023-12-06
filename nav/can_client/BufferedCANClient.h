#pragma once

#include "nav/can_client/CANDBC.h"
#include "cyber/common/log.h"

#include <units.h>
#include <boost/circular_buffer.hpp>

#include <chrono>
#include <vector>

namespace nav {
    namespace can {

        template <class CANDevice>
        class BufferedCANClient {
        public:
            BufferedCANClient(std::unique_ptr<CANDevice> canDevice);

            /// Waits for CAN messages to appear
            bool waitForMessages(units::time::millisecond_t timeout = units::time::millisecond_t(3000));

            /// Attempts to read a single message from the bus
            void readMessages();

            /// Attempts to send a single CAN message
            int32_t sendMessage(const CANMessage message, bool forceExtended = false);

            /// Attempts to send a queue of messages
            int32_t sendMessageQueue(std::vector<CANMessage> messages,
                                     units::time::millisecond_t delay = units::time::millisecond_t(20), bool forceExtended = false);

            /// Attempts to read all queued messages from the bus
            std::vector<CANMessage> getQueuedMessages();

            /// Attempts to set a new CAN filter mask to the BIOS
            void setCanFilterMask(const int32_t mask);
        private:
            std::unique_ptr<CANDevice> canDevice_;
            boost::circular_buffer<uint8_t> circularBuffer_;
        };

        template <class CANDevice>
        BufferedCANClient<CANDevice>::BufferedCANClient(std::unique_ptr<CANDevice> canDevice) :
        canDevice_{std::move(canDevice)}
        {

        }

        template <class CANDevice>
        void BufferedCANClient<CANDevice>::readMessages() {

        }

        template <class CANDevice>
        std::vector<CANMessage> BufferedCANClient<CANDevice>::getQueuedMessages() {

        }

    } // namespace can
} // namespace nav


