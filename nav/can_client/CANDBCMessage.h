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
    std::vector<CANDBCSignal> signals_;
};

} // namespace can
} // namespace nav
