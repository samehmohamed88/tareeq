#include "nav/can_client/CANDBC.h"

#include <vector>
#include <cstdint>
#include <string>
#include <set>
#include <regex>
#include <optional>


namespace nav {
namespace can {

std::optional<std::reference_wrapper<const MessageSchema>> CANDBC::getMessageByAddress(uint32_t address) {
    const auto it = messagesAddressMap_.find(address);
    if (it != messagesAddressMap_.end()) {
        return std::cref(it->second);
    } else {
        return std::nullopt; // Represents an empty optional
    }
}

std::optional<std::reference_wrapper<const MessageSchema>> CANDBC::getMessageByName(std::string messageName) {
    const auto it = messagesNameMap_.find(messageName);
    if (it != messagesNameMap_.end()) {
        return std::cref(it->second);
    } else {
        return std::nullopt; // Represents an empty optional
    }
}

std::optional<std::reference_wrapper<const std::vector<SignalSchema>>> CANDBC::getSignalSchemasByAddress(uint32_t address) {
    const auto it = messagesAddressMap_.find(address);
    if (it != messagesAddressMap_.end()) {
        return it->second.signals;
    } else {
        return std::nullopt; // Represents an empty optional
    }
}

std::string& CANDBC::trim(std::string& s, const char* t) {
    s.erase(s.find_last_not_of(t) + 1);
    return s.erase(0, s.find_first_not_of(t));
}

bool CANDBC::startswith(const std::string& str, const char* prefix) {
    return str.find(prefix, 0) == 0;
}

bool CANDBC::startswith(const std::string& str, std::initializer_list<const char*> prefix_list) {
    for (auto prefix : prefix_list) {
        if (startswith(str, prefix)) return true;
    }
    return false;
}

bool CANDBC::endswith(const std::string& str, const char* suffix) {
    return str.find(suffix, 0) == (str.length() - strlen(suffix));
}

}
}
