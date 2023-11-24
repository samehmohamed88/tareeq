#pragma once

#include <cstdint>
#include <string>

struct __attribute__((packed)) CANHeader {
    uint8_t reserved : 1;
    uint8_t bus : 3;
    uint8_t data_len_code : 4;
    uint8_t rejected : 1;
    uint8_t returned : 1;
    uint8_t extended : 1;
    uint32_t addr : 29;
    uint8_t checksum : 8;
};

struct CANFrame {
    long address;
    std::string dat;
    long busTime;
    long src;
};

