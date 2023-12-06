#include "nav/can_client/SocketCANDeviceImpl.h"
#include "nav/can_client/exceptions/CANInitException.h"
#include "nav/can_client/exceptions/CANCloseException.h"

#include "unistd.h"

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <cstring>

namespace nav {
    namespace can {

        void SocketCANDeviceImpl::initDevice() {
            struct sockaddr_can address;
            struct ifreq ifaceRequest;
            int64_t fdOptions = 0;
            int32_t tmpReturn;

            memset(&address, 0, sizeof(sizeof(struct sockaddr_can)));
            memset(&ifaceRequest, 0, sizeof(sizeof(struct ifreq)));

            socketFd_ = socket(PF_CAN, SOCK_RAW, canProtocol_);

            if (socketFd_ == -1) {
                throw exception::CANInitException(formatString("FAILED to initialise socketcan! Error: %d => %s", errno, strerror(errno)));
            }
            strcpy(ifaceRequest.ifr_name, interfaceName_.c_str());

            if ((tmpReturn = ioctl(socketFd_, SIOCGIFINDEX, &ifaceRequest)) == -1) {
                throw exception::CANInitException(formatString("FAILED to perform IO control operation on socket %s! Error: %d => %s", interfaceName_.c_str(), errno,
                                                    strerror(errno)));
            }
//            fdOptions = fcntl(socketFd_, F_GETFL);
//            fdOptions |= O_NONBLOCK;
//            tmpReturn = fcntl(socketFd_, F_SETFL, fdOptions);

            address.can_family = PF_CAN;
            address.can_ifindex = ifaceRequest.ifr_ifindex;

//            setCanFilterMask(_canFilterMask);

            if ((tmpReturn = bind(socketFd_, (struct sockaddr*)&address, sizeof(address))) == -1) {
                throw exception::CANInitException(formatString("FAILED to bind to socket CAN! Error: %d => %s", errno, strerror(errno)));
            }

            /*Define receive filter rules,we can set more than one filter rule!*/
//            struct can_filter rfilter[2];
//            rfilter[0].can_id = 0x123;//Standard frame id !
//            rfilter[0].can_mask = CAN_SFF_MASK;
//            rfilter[1].can_id = 0x12345678;//extend frame id!
//            rfilter[1].can_mask = CAN_EFF_MASK;
            //mask below sentense to receive all the message from CAN BUS!
            //setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

        }

        ssize_t SocketCANDeviceImpl::readSocket(void *buf, size_t count) const {
            return read(socketFd_, buf, count);
        }

        ssize_t SocketCANDeviceImpl::writeToSocket(const void *buf, size_t count) const {
            return write(socketFd_, buf, count);
        }

        int SocketCANDeviceImpl::close(int fd) const {
            return close(fd);
        }
    }
}
