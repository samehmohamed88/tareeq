#include "nav/can_client/CANDBC.h"
#include "nav/can_client/SubaruGlobalCANDBC.h"

#include <vector>
#include <cstdint>
#include <string>
#include <set>
#include <map>
#include <sstream>
#include <regex>
#include <optional>


namespace nav {
namespace can {

std::optional<std::reference_wrapper<const CANDBC::MessageSchema>> CANDBC::getMessageByAddress(uint32_t address) {
    const auto it = messagesAddressMap_.find(address);
    if (it != messagesAddressMap_.end()) {
        return std::cref(it->second);
    } else {
        return std::nullopt; // Represents an empty optional
    }
}

std::optional<std::reference_wrapper<const std::vector<CANDBC::SignalSchema>>> CANDBC::getSignalSchemasByAddress(uint32_t address) {
    const auto it = messagesAddressMap_.find(address);
    if (it != messagesAddressMap_.end()) {
        return it->second.signals;
    } else {
        return std::nullopt; // Represents an empty optional
    }
}

void CANDBC::parseMessageLine_BO_(){

}

inline std::string& CANDBC::trim(std::string& s, const char* t) {
    s.erase(s.find_last_not_of(t) + 1);
    return s.erase(0, s.find_first_not_of(t));
}



inline bool CANDBC::startswith(const std::string& str, const char* prefix) {
    return str.find(prefix, 0) == 0;
}

inline bool CANDBC::startswith(const std::string& str, std::initializer_list<const char*> prefix_list) {
    for (auto prefix : prefix_list) {
        if (startswith(str, prefix)) return true;
    }
    return false;
}

inline bool CANDBC::endswith(const std::string& str, const char* suffix) {
    return str.find(suffix, 0) == (str.length() - strlen(suffix));
}

CANDBC::CANDBC(bool allow_duplicate_message_name) {
    /// The variable `subaruGlobalCANDBC` is defined in header `nav/can_client/SubaruGlobalCANDBC.h` which is generated
    /// by the bazel `genrule(subaru_global_can_dbc)` in the `BUILD` file.
    std::istringstream dbcStream(subaruGlobalCANDBC);

    uint32_t address = 0;
    std::set<uint32_t> addressSet;
    std::set<std::string> messageNameSet;
    std::map<uint32_t, std::set<std::string>> signalNameSetMap;
    std::map<uint32_t, std::vector<SignalSchema>> signalAddressMap;
    std::setlocale(LC_NUMERIC, "C");

    // used to find big endian LSB from MSB and size
    std::vector<int> bigEndianBits;
    for (int i = 0; i < 64; i++) {
        for (int j = 7; j >= 0; j--) {
            bigEndianBits.push_back(j + i * 8);
        }
    }

    std::string line;
    int line_num = 0;
    std::smatch match;
    // TODO: see if we can speed up the regex statements in this loop, SG_ is specifically the slowest
    while (std::getline(dbcStream, line)) {
        line = trim(line);
        line_num += 1;
        if (startswith(line, "BO_ ")) {
            // new group
            bool ret = std::regex_match(line, match, boRegExp_);
            DBC_ASSERT(ret, "bad BO: " << line);

            address = std::stoul(match[1].str());
            auto result = messagesAddressMap_.emplace(address, MessageSchema{});

            if (result.second) { // Check if insertion took place
                MessageSchema& message = result.first->second;
                message.address = std::stoul(match[1].str());  // could be hex
                message.name = match[2].str();
                message.size = std::stoul(match[3].str());

                // check for duplicates
                DBC_ASSERT(addressSet.find(address) == addressSet.end(), "Duplicate message address: " << address << " (" << message.name << ")");
                addressSet.insert(address);

                if (!allow_duplicate_message_name) {
                    DBC_ASSERT(messageNameSet.find(message.name) == messageNameSet.end(), "Duplicate message name: " << message.name);
                    messageNameSet.insert(message.name);
                }
            }
        } else if (startswith(line, "SG_ ")) {
            // new signal
            int offset = 0;
            if (!std::regex_search(line, match, sgRegExp_)) {
                bool ret = std::regex_search(line, match, sgmRegExp_);
                DBC_ASSERT(ret, "bad SG: " << line);
                offset = 1;
            }
            SignalSchema& signalSchema = signalAddressMap[address].emplace_back();
            signalSchema.name = match[1].str();
            signalSchema.startBit = std::stoi(match[offset + 2].str());
            signalSchema.size = std::stoi(match[offset + 3].str());
            signalSchema.isLittleEndian = std::stoi(match[offset + 4].str()) == 1;
            signalSchema.is_signed = match[offset + 5].str() == "-";
            signalSchema.factor = std::stod(match[offset + 6].str());
            signalSchema.offset = std::stod(match[offset + 7].str());
            if (signalSchema.isLittleEndian) {
                signalSchema.leastSignificantBit = signalSchema.startBit;
                signalSchema.mostSignificantBit = signalSchema.startBit + signalSchema.size - 1;
            } else {
                auto it = std::find(bigEndianBits.begin(), bigEndianBits.end(), signalSchema.startBit);
                signalSchema.leastSignificantBit = bigEndianBits[(it - bigEndianBits.begin()) + signalSchema.size - 1];
                signalSchema.mostSignificantBit = signalSchema.startBit;
            }
            DBC_ASSERT(signalSchema.leastSignificantBit < (64 * 8) && signalSchema.mostSignificantBit < (64 * 8), "Signal out of bounds: " << line);

            // Check for duplicate signal names
            DBC_ASSERT(signalNameSetMap[address].find(signalSchema.name) == signalNameSetMap[address].end(), "Duplicate signal name: " << signalSchema.name);
            signalNameSetMap[address].insert(signalSchema.name);
        } else if (startswith(line, "VAL_ ")) {
            // new signal value/definition
            bool ret = std::regex_search(line, match, valRegExp_);
            DBC_ASSERT(ret, "bad VAL: " << line);

            address = std::stoul(match[1].str());
            auto name = match[2].str();
            auto valueDefinitions = match[3].str();
            std::sregex_token_iterator it{valueDefinitions.begin(), valueDefinitions.end(), valSplitRegExp_, -1};
            // convert strings to UPPER_CASE_WITH_UNDERSCORES
            std::vector<std::string> words{it, {}};
            for (auto& w : words) {
                w = trim(w);
                std::transform(w.begin(), w.end(), w.begin(), ::toupper);
                std::replace(w.begin(), w.end(), ' ', '_');
            }
            // join string
            std::stringstream s;
            std::copy(words.begin(), words.end(), std::ostream_iterator<std::string>(s, " "));
            valueDefinitions = s.str();
            valueDefinitions = trim(valueDefinitions);


            auto &signals = signalAddressMap[address];
            for (auto signal : signals) {
                if (signal.name == name) {
                    signal.valueDescription_ = SignalSchema::ValueDescription{
                            name,
                            address,
                            valueDefinitions
                    };
                }
            }
        }
    }
    for (auto& [key, message] : messagesAddressMap_) {
        message.signals = signalAddressMap[key];
    }
}

}
}
