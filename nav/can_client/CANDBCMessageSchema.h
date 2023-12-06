#pragma once

#include "nav/can_client/CANDBCSignalSchema.h"

#include <string>
#include <vector>
#include <unordered_map>
#include <functional>

namespace nav {
namespace can {

class CANDBCMessageSchema {
public:
    const std::string &getName() const;
    uint32_t getAddress();
    uint32_t getSize();
    std::optional<std::reference_wrapper<const CANDBCSignalSchema>> getSignalSchemaByName(std::string signalName) const;
private:
    std::string name_;
    uint32_t address_;
    uint32_t size_;
    std::vector<CANDBCSignalSchema> signals_;
    std::unordered_map<std::string, const CANDBCSignalSchema> signalNameToSignalMap;
};
} // namespace can
} // namespace nav