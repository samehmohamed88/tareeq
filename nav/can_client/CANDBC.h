#pragma once

#include <vector>
#include <cstdint>
#include <string>
#include <map>
#include <regex>
#include <optional>

namespace nav {
namespace can {

struct __attribute__((packed)) CANHeader {
    uint8_t reserved : 1;
    uint8_t bus : 3;
    uint8_t data_len_code : 4;
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
    struct SignalSchema {
        struct ValueDescription {
            std::string name;
            uint32_t address;
            std::string def_val;
        };
        std::string name;
        int startBit, mostSignificantBit, leastSignificantBit, size;
        bool is_signed;
        double factor, offset;
        bool isLittleEndian;
        unsigned int (*calcChecksum)(uint32_t address, const SignalSchema &signal, const std::vector<uint8_t> &d);
        ValueDescription valueDescription_;
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

    CANDBC(bool allow_duplicate_message_name = false);

    void parseMessageLine_BO_();
    inline std::string& trim(std::string& s, const char* t = " \t\n\r\f\v");
    inline bool startswith(const std::string& str, const char* prefix);
    inline bool startswith(const std::string& str, std::initializer_list<const char*> prefix_list);
    inline bool endswith(const std::string& str, const char* suffix);

///
#define DBC_ASSERT(condition, message)                             \
  do {                                                             \
    if (!(condition)) {                                            \
      std::stringstream is;                                        \
      is << "[" << name_ << ":" << line_num << "] " << message; \
      throw std::runtime_error(is.str());                          \
    }                                                              \
  } while (false)
};
}
}
