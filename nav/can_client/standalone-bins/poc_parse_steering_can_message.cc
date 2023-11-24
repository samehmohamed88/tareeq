#include <string>
#include <vector>

///
/// \return

struct Signal {
    std::string name;
    int start_bit, msb, lsb, size;
    bool is_signed;
    double factor, offset;
    bool is_little_endian;
//    SignalType type;
    unsigned int (*calc_checksum)(uint32_t address, const Signal &sig, const std::vector<uint8_t> &d);
};

struct Msg {
    std::string name;
    uint32_t address;
    unsigned int size;
    std::vector<Signal> sigs;
};

struct Val {
    std::string name;
    uint32_t address;
    std::string def_val;
    std::vector<Signal> sigs;
};

struct DBC {
    std::string name;
    std::vector<Msg> msgs;
    std::vector<Val> vals;
};

void can_parser_parse() {

}



constexpr auto vectorSize = 0x4000U;
constexpr uint8_t dataLengthCodeToNumBytes[] = {0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U};

int main() {

}