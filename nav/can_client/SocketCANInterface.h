#pragma once

#include <cstdio>

namespace nav {
namespace can {
class SocketCANInterface {
public:
    virtual ssize_t readSocket(void *buf, size_t count) const = 0;
    virtual ssize_t writeToSocket(const void *buf, size_t count) const = 0;
    virtual int close(int fd) const = 0;
};
}
}