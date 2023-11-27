#pragma once

#include "nav/can_client/SubaruGlobalCANDBC.h"

#include <vector>
#include <cstdint>
#include <string>
#include <map>
#include <regex>
#include <optional>
#include <set>

namespace nav {
namespace can {

struct __attribute__((packed)) CANHeader {
    uint8_t reserved : 1;
    uint8_t bus : 3;
    uint8_t dataLengthCode : 4;
    uint8_t rejected : 1;
    uint8_t returned : 1;
    uint8_t extended : 1;
    uint32_t addr : 29;
    uint8_t checksum : 8;
};

constexpr size_t getSizeOfCANHeader() {
    return sizeof(CANHeader);
}

struct CANFrame {
    long address;
    std::vector<uint8_t> data;
    long busTime;
    long src;
    uint8_t counter = 0;
    uint8_t numCounterErrors = 0;
};

struct CANMessage {
    struct CANSignal {
        uint64_t timestampNanoSeconds;
        std::string name;
        double value;  // latest value
        // does not seem useful at the moment
//         std::vector<double> all_values;  // all values from this cycle
    };
    uint32_t address;
    std::string name;
    std::vector<CANSignal> signals;
};

class CANDBC {
public:
    // we define these below
    struct MessageSchema;
    struct SignalSchema;

    static constexpr uint8_t dataLengthCodeToNumBytes[] = {0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U};

    static std::unique_ptr<CANDBC> CreateInstance() {
        return std::move(std::make_unique<CANDBC>(CANDBC()));
    }

    std::optional<std::reference_wrapper<const MessageSchema>> getMessageByAddress(uint32_t address);

    std::optional<std::reference_wrapper<const std::vector<SignalSchema>>> getSignalSchemasByAddress(uint32_t address);
public:
    enum class SignalType {
        DEFAULT,
        COUNTER,
        CHECKSUM,
    };
    /// This is taken from CommaAI openpilot
    /// This project only supports Subaru Forester 2020 whereas Comma AI supports many different cars.
    /// So we hardcode the values as they are in openpilot for Subaru
    /// https://github.com/commaai/opendbc/blob/2b96bcc45669cdd14f9c652b07ef32d6403630f6/can/dbc.cc#L65
    struct ChecksumState {
        static const int checksumSize = 8;
        static const int counterSize = -1;
        static const int checksumStartBit = 0;
        static const int counterStartBit = -1;
        static const bool littleEndian = true;
    };
    struct SignalSchema {
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
    };
    struct MessageSchema {
        std::string name;
        uint32_t address;
        uint32_t size;
        std::vector<SignalSchema> signals;
    };
private:
    ///
    std::regex boRegExp_{R"(^BO_ (\w+) (\w+) *: (\w+) (\w+))"};
    std::regex sgRegExp_{R"(^SG_ (\w+) : (\d+)\|(\d+)@(\d+)([\+|\-]) \(([0-9.+\-eE]+),([0-9.+\-eE]+)\) \[([0-9.+\-eE]+)\|([0-9.+\-eE]+)\] \"(.*)\" (.*))"};
    std::regex sgmRegExp_{R"(^SG_ (\w+) (\w+) *: (\d+)\|(\d+)@(\d+)([\+|\-]) \(([0-9.+\-eE]+),([0-9.+\-eE]+)\) \[([0-9.+\-eE]+)\|([0-9.+\-eE]+)\] \"(.*)\" (.*))"};
    std::regex valRegExp_{R"(VAL_ (\w+) (\w+) (\s*[-+]?[0-9]+\s+\".+?\"[^;]*))"};
    std::regex valSplitRegExp_{R"([\"]+)"};  // split on "

    std::string name_ = std::string("Subaru Forester 2020");
    std::map<uint32_t, MessageSchema> messagesAddressMap_;

    std::string& trim(std::string& s, const char* t = " \t\n\r\f\v");
    bool startswith(const std::string& str, const char* prefix);
    bool startswith(const std::string& str, std::initializer_list<const char*> prefix_list);
    bool endswith(const std::string& str, const char* suffix);

///
#define DBC_ASSERT(condition, message)                             \
  do {                                                             \
    if (!(condition)) {                                            \
      std::stringstream is;                                        \
      is << "[" << name_ << ":" << line_num << "] " << message; \
      throw std::runtime_error(is.str());                          \
    }                                                              \
  } while (false)



    CANDBC(bool allow_duplicate_message_name = false) {
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
                signalSchema.messageName = messagesAddressMap_[address].name;
                signalSchema.startBit = std::stoi(match[offset + 2].str());
                signalSchema.size = std::stoi(match[offset + 3].str());
                signalSchema.isLittleEndian = std::stoi(match[offset + 4].str()) == 1;
                signalSchema.is_signed = match[offset + 5].str() == "-";
                signalSchema.factor = std::stod(match[offset + 6].str());
                signalSchema.offset = std::stod(match[offset + 7].str());

                if (signalSchema.name == "CHECKSUM") {
                    DBC_ASSERT(ChecksumState::checksumSize == -1 || signalSchema.size == ChecksumState::checksumSize, "CHECKSUM is not " << ChecksumState::checksumSize << " bits long");
                    DBC_ASSERT(ChecksumState::checksumStartBit == -1 || (signalSchema.startBit % 8) == ChecksumState::checksumStartBit, " CHECKSUM starts at wrong bit");
                    DBC_ASSERT(signalSchema.isLittleEndian == ChecksumState::littleEndian, "CHECKSUM has wrong endianness");
                    signalSchema.type = SignalType::CHECKSUM;
                } else if (signalSchema.name == "COUNTER") {
                    DBC_ASSERT(ChecksumState::counterSize == -1 || signalSchema.size == ChecksumState::counterSize, "COUNTER is not " << ChecksumState::counterSize << " bits long");
                    DBC_ASSERT(ChecksumState::counterStartBit == -1 || (signalSchema.startBit % 8) == ChecksumState::counterStartBit, "COUNTER starts at wrong bit");
                    DBC_ASSERT(ChecksumState::littleEndian == signalSchema.isLittleEndian, "COUNTER has wrong endianness");
                    signalSchema.type = SignalType::COUNTER;
                }

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

};
}
}
