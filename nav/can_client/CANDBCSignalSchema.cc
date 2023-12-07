#include "nav/can_client/CANDBCSignalSchema.h"

namespace nav {
namespace can {
/// This is taken from CommaAI openpilot
/// reference: https://github.com/commaai/opendbc/blob/2b96bcc45669cdd14f9c652b07ef32d6403630f6/can/common.cc#L27C1-L27C1
uint8_t CANDBCSignalSchema::calcSubaruChecksum(uint32_t address, const std::vector<uint8_t> &d) const {
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

int64_t CANDBCSignalSchema::parseValue(const std::vector<uint8_t> &messageData) const {
    int64_t ret = 0;
    int i = mostSignificantBit_ / 8;
    int bits = size_;
    while (i >= 0 && i < static_cast<int>(messageData.size()) && bits > 0) {
        int lsb = (int)(leastSignificantBit_ / 8) == i ? leastSignificantBit_ : i*8;
        int msb = (int)(mostSignificantBit_ / 8) == i ? mostSignificantBit_ : (i+1)*8 - 1;
        int size = msb - lsb + 1;

        uint64_t d = (messageData[i] >> (lsb - (i*8))) & ((1ULL << size) - 1);
        ret |= d << (bits - size);

        bits -= size;
        i = isLittleEndian() ? i-1 : i+1;
    }
    return ret;
}

void CANDBCSignalSchema::packValue(std::vector<uint8_t> &writeBuffer, int64_t tmp_val) const {
    int i = leastSignificantBit_ / 8;
    int bits = size_;
    if (size_ < 64) {
        tmp_val &= ((1ULL << size_) - 1);
    }

    while (i >= 0 && i < static_cast<int>(writeBuffer.size()) && bits > 0) {
        int shift = (int)(leastSignificantBit_ / 8) == i ? leastSignificantBit_ % 8 : 0;
        int size = std::min(bits, 8 - shift);

        writeBuffer[i] &= ~(((1ULL << size) - 1) << shift);
        writeBuffer[i] |= (tmp_val & ((1ULL << size) - 1)) << shift;

        bits -= size;
        tmp_val >>= size;
        i = isLittleEndian() ? i+1 : i-1;
    }
}

const std::string &CANDBCSignalSchema::getName() const {
    return name_;
}

void CANDBCSignalSchema::setName(std::string name) {
    name_ = name;
}

const std::string &CANDBCSignalSchema::getMessageName() const {
    return messageName_;
}

int CANDBCSignalSchema::getStartBit() const {
    return startBit_;
}

int CANDBCSignalSchema::getMostSignificantBit() const {
    return mostSignificantBit_;
}

int CANDBCSignalSchema::getLeastSignificantBit() const {
    return leastSignificantBit_;
}

int CANDBCSignalSchema::getSize() const {
    return size_;
}
bool CANDBCSignalSchema::isSigned() const {
    return isSigned_;
}

double CANDBCSignalSchema::getFactor() const {
    return factor_;
}
double CANDBCSignalSchema::getOffset() const {
    return offset_;
}

bool CANDBCSignalSchema::isLittleEndian() const {
    return isLittleEndian_;
}

const SignalType &CANDBCSignalSchema::getSignalType() const {
    return type_;
}

void CANDBCSignalSchema::setMessageName(const std::string &messageName) {
    messageName_ = messageName;
}

void CANDBCSignalSchema::setStartBit(int startBit) {
    startBit_ = startBit;
}

void CANDBCSignalSchema::setMostSignificantBit(int mostSignificantBit) {
    mostSignificantBit_ = mostSignificantBit;
}

void CANDBCSignalSchema::setLeastSignificantBit(int leastSignificantBit) {
    leastSignificantBit_ = leastSignificantBit;
}

void CANDBCSignalSchema::setSize(int size) {
    size_ = size;
}

void CANDBCSignalSchema::setIsSigned(bool isSigned) {
    isSigned_ = isSigned;
}

void CANDBCSignalSchema::setFactor(double factor) {
    factor_ = factor;
}

void CANDBCSignalSchema::setOffset(double offset) {
    offset_ = offset;
}

void CANDBCSignalSchema::setIsLittleEndian(bool isLittleEndian) {
    isLittleEndian_ = isLittleEndian;
}

SignalType CANDBCSignalSchema::getType() const {
    return type_;
}

void CANDBCSignalSchema::setType(SignalType type) {
    type_ = type;
}

const CANDBCSignalSchema::ValueDescription &CANDBCSignalSchema::getValueDescription() const {
    return valueDescription_;
}

void CANDBCSignalSchema::setValueDescription(const CANDBCSignalSchema::ValueDescription &valueDescription) {
    valueDescription_ = valueDescription;
}

}
}