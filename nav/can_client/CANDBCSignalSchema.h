#pragma once

#include <string>
#include <vector>

namespace nav {
namespace can {
enum class SignalType {
    DEFAULT,
    COUNTER,
    CHECKSUM,
};
class CANDBCSignalSchema {
public:
    struct ValueDescription {
        std::string name;
        uint32_t address;
        std::string def_val;
    };
    std::string name;
    std::string messageName;
    int startBit, mostSignificantBit, leastSignificantBit, size;
    bool is_signed;
    double factor, offset;
    bool isLittleEndian;
    SignalType type;
    ValueDescription valueDescription_;
    /// This is taken from CommaAI openpilot
    /// reference: https://github.com/commaai/opendbc/blob/2b96bcc45669cdd14f9c652b07ef32d6403630f6/can/common.cc#L27C1-L27C1
    uint8_t calcSubaruChecksum(uint32_t address, const std::vector<uint8_t> &d) const {
        unsigned int s = 0;
        while (address) {
            s += address & 0xFF; address >>= 8;
        }
        // skip checksum in first byte
        for (size_t i = 1; i < d.size(); i++) {
            s += d[i];
        }
        return s & 0xFF;
    };

    int64_t parseValue(const std::vector<uint8_t> &messageData) {
        int64_t ret = 0;
        int i = mostSignificantBit / 8;
        int bits = size;
        while (i >= 0 && i < static_cast<int>(messageData.size()) && bits > 0) {
            int lsb = (int)(leastSignificantBit / 8) == i ? leastSignificantBit : i*8;
            int msb = (int)(mostSignificantBit / 8) == i ? mostSignificantBit : (i+1)*8 - 1;
            int size = msb - lsb + 1;

            uint64_t d = (messageData[i] >> (lsb - (i*8))) & ((1ULL << size) - 1);
            ret |= d << (bits - size);

            bits -= size;
            i = isLittleEndian ? i-1 : i+1;
        }
        return ret;
    }

    void packValue(std::vector<uint8_t> &writeBuffer, int64_t tmp_val) const {
        int i = leastSignificantBit / 8;
        int bits = size;
        if (size < 64) {
            tmp_val &= ((1ULL << size) - 1);
        }

        while (i >= 0 && i < static_cast<int>(writeBuffer.size()) && bits > 0) {
            int shift = (int)(leastSignificantBit / 8) == i ? leastSignificantBit % 8 : 0;
            int size = std::min(bits, 8 - shift);

            writeBuffer[i] &= ~(((1ULL << size) - 1) << shift);
            writeBuffer[i] |= (tmp_val & ((1ULL << size) - 1)) << shift;

            bits -= size;
            tmp_val >>= size;
            i = isLittleEndian ? i+1 : i-1;
        }
    }
};


}
}