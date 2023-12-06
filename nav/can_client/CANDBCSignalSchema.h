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
    const std::string &getName() const;
    const std::string &getMessageName() const;
    int getStartBit() const;
    int getMostSignificantBit() const;
    int getLeastSignificantBit() const;
    int getSize() const;
    bool isSigned() const;
    double getFactor() const;
    double getOffset() const;
    bool isLittleEndian() const;
    const SignalType &getSignalType() const;
    uint8_t calcSubaruChecksum(uint32_t address, const std::vector<uint8_t> &d) const;
    int64_t parseValue(const std::vector<uint8_t> &messageData);

    void setName(std::string name);

    void setMessageName(const std::string &messageName);

    void setStartBit(int startBit);

    void setMostSignificantBit(int mostSignificantBit);

    void setLeastSignificantBit(int leastSignificantBit);

    void setSize(int size);

    void setIsSigned(bool isSigned);

    void setFactor(double factor);

    void setOffset(double offset);

    void setIsLittleEndian(bool isLittleEndian);

    SignalType getType() const;

    void setType(SignalType type);

    const ValueDescription &getValueDescription() const;

    void setValueDescription(const ValueDescription &valueDescription);

    void packValue(std::vector<uint8_t> &writeBuffer, int64_t tmp_val) const;
private:
    std::string name_;
    std::string messageName_;
    int startBit_;
    int mostSignificantBit_;
    int leastSignificantBit_ ;
    int size_;
    bool isSigned_;
    double factor_;
    double offset_;
    bool isLittleEndian_;
    SignalType type_;
    ValueDescription valueDescription_;

};


}
}