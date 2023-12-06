#pragma once

#include <vector>
#include <string>

namespace nav {
namespace can {

// forward declarations
enum class CANBus;

class CANDBCSignal {
public:
    const std::string& getName() const;
    double getValue() const;
private:
public:
    uint64_t getTimestampNanoSeconds() const;

    void setTimestampNanoSeconds(uint64_t timestampNanoSeconds);

    void setName(const std::string &name);

    void setValue(double value);

private:
    uint64_t timestampNanoSeconds;
    std::string name_;
    double value_;  // latest value
    // does not seem useful at the moment
//         std::vector<double> all_values;  // all values from this cycle
};

class CANDBCMessage {
public:
    uint32_t getAddress() const;
    const std::string& getName() const;
    const CANBus& getCANBus() const;
    const std::vector<CANDBCSignal>& getSignals() const;
private:
    uint32_t address_;
    std::string name_;
    CANBus canBus_;
public:
    void setAddress(uint32_t address);

    void setName(const std::string &name);

    CANBus getCanBus() const;

    void setCanBus(CANBus canBus);

    void setSignals(const std::vector<CANDBCSignal> &signals);

private:
    std::vector<CANDBCSignal> signals_;
};

} // namespace can
} // namespace nav
