#pragma once

#include <cstdio>

namespace nav {
    namespace can {
        class SocketCANInterface {
        public:
            virtual ssize_t read(int fd, void *buf, size_t count) const;
            virtual ssize_t write(int fd, const void *buf, size_t count) const;
        };
    }
}