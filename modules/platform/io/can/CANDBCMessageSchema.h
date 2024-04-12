#pragma once

#include "can_client/CANDBCSignalSchema.h"

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

namespace nav {
namespace can {

class CANDBCMessageSchema {
public:
    const std::string &getName() const;
    void setName(std::string name);
    uint32_t getAddress();
    void setAddress(uint32_t address);
    uint32_t getSize() const;
    void setSize(uint32_t size);
    std::optional<std::reference_wrapper<const CANDBCSignalSchema>> getSignalSchemaByName(std::string signalName) const;
private:
    std::string name_;
    uint32_t address_;
    uint32_t size_;
public:
    void moveSignalSchemaToMap(std::string signalSchemaName, CANDBCSignalSchema signalSchema);

    const std::unordered_map<std::string, const CANDBCSignalSchema> &getSignalNameToSignalMap() const;

    void
    setSignalNameToSignalMap(const std::unordered_map<std::string, const CANDBCSignalSchema> &signalNameToSignalMap);

public:
    const std::vector<CANDBCSignalSchema> &getSignals() const;

    void setSignals(const std::vector<CANDBCSignalSchema> &signals);

private:
    std::vector<CANDBCSignalSchema> signals_;
    std::unordered_map<std::string, const CANDBCSignalSchema> signalNameToSignalMap_;
};
} // namespace can
} // namespace nav