#pragma once

#include "nav/can_client/SocketCANMessage.h"
#include "cyber/common/log.h"

#include <memory>
#include <mutex>
#include <vector>

namespace nav {
namespace can {
template <class CANSocket>
class SocketCANDevice {
public:
    SocketCANDevice(std::unique_ptr<CANSocket> socket);
    SocketCANMessage getMessage();
    void sendMessage(uint32_t address, std::vector<uint8_t> rawData);
    void sendMessage(SocketCANMessage messageToSend);
private:
    void sendMessage(const can_frame &frameToSend);
private:
    std::unique_ptr<CANSocket> socket_;
    std::mutex mutex_;
};

template <class CANSocket>
SocketCANDevice<CANSocket>::SocketCANDevice(std::unique_ptr<CANSocket> socket) :
    socket_{std::move(socket)}
{
    socket_->initDevice();
}

template <class CANSocket>
SocketCANMessage SocketCANDevice<CANSocket>::getMessage() {
    std::lock_guard<std::mutex> lock(mutex_);
    // actual number of bytes read
    int32_t numBytesRead{0};
    // struct can_frame with can Id and data array
    can_frame canFrame;
    memset(&canFrame, 0, sizeof(can_frame));
    // read socket can
    numBytesRead = socket_->readSocket(&canFrame, sizeof(canFrame));
    if (0 > numBytesRead) {
        AERROR << "Failed to read CAN Message "
               << errno << " " << strerror(errno);
        // we make a new frame and set its content to zeros just in case the readSocket
        // put any data on the `canFrame`.
        can_frame errorFrame;
        memset(&errorFrame, 0, sizeof(can_frame));
        return SocketCANMessage(errorFrame);
    }
    return SocketCANMessage{canFrame};
};

template <class CANSocket>
void SocketCANDevice<CANSocket>::sendMessage(const can_frame &frameToSend) {
    int numBytesWritten = socket_->write(&frameToSend, sizeof(frameToSend));
}

template <class CANSocket>
void SocketCANDevice<CANSocket>::sendMessage(uint32_t address, std::vector<uint8_t> rawData) {
//    socket_->
}

template <class CANSocket>
void SocketCANDevice<CANSocket>::sendMessage(SocketCANMessage messageToSend) {
    sendMessage(messageToSend.getRawFrame());
}


} // namespace can
} // namespace nav