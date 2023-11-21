#pragma once

#include "l3p/can_client/UsbCANAdapter.h"

#include <cstdint>
#include <ctime>
#include <functional>
#include <list>
#include <memory>
#include <optional>
#include <string>
#include <vector>

//#include "cereal/gen/cpp/car.capnp.h"
//#include "cereal/gen/cpp/log.capnp.h"
//#include "panda/board/health.h"
//#include "panda/board/can_definitions.h"
//#include "selfdrive/boardd/panda_comms.h"



namespace l3p {
namespace can {
#define USB_TX_SOFT_LIMIT   (0x100U)
#define USBPACKET_MAX_SIZE  (0x40)

#define RECV_SIZE (0x4000U)

#define CAN_REJECTED_BUS_OFFSET   0xC0U
#define CAN_RETURNED_BUS_OFFSET 0x80U





template <class CANAdapter>
class CANClient {
public:
//  Panda(std::string serial="", uint32_t bus_offset=0);
//    Panda();
    CANClient();
//    cereal::PandaState::PandaType hw_type = cereal::PandaState::PandaType::UNKNOWN;
//    bool has_rtc = false;

    /// Checks if the USB Adapter is connected
    bool isConnected() const;
    /// Checks if the communication with the USB Adapter is healthy
    bool isCommHealthy() const;

//    std::string hw_serial();

    // Static functions
//    static std::vector<std::string> list(bool usb_only=false);

    // Panda functionality
//    cereal::PandaState::PandaType get_hw_type();

    /// Sets the safety model on the CAN Device
    /// The Comma AI CAN Device has a specific safety model where it starts in silent mode with No Output to prevent
    /// accidental movements/steering
    /// taken from https://github.com/commaai/cereal/blob/416c3d531c90ce16498d782bf383625a857ee74c/car.capnp#L567C8-L567C19
    void setSafetyModel(SafetyModel safety_model, uint16_t safety_param=0U);
//    void set_alternative_experience(uint16_t alternative_experience);
//    void set_rtc(struct tm sys_time);
//    struct tm get_rtc();
//    void set_fan_speed(uint16_t fan_speed);
//    uint16_t get_fan_speed();
//    void set_ir_pwr(uint16_t ir_pwr);
//    std::optional<health_t> get_state();
//    std::optional<can_health_t> get_can_state(uint16_t can_number);
//    void set_loopback(bool loopback);
//    std::optional<std::vector<uint8_t>> get_firmware_version();
//    bool up_to_date();
//    std::optional<std::string> get_serial();
//    void set_power_saving(bool power_saving);
//    void enable_deepsleep();
//    void send_heartbeat(bool engaged);
//    void set_can_speed_kbps(uint16_t bus, uint16_t speed);
//    void set_data_speed_kbps(uint16_t bus, uint16_t speed);
//    void set_canfd_non_iso(uint16_t bus, bool non_iso);
//    void can_send(capnp::List<cereal::CanData>::Reader can_data_list);
//    bool can_receive(std::vector<can_frame>& out_vec);

    /// Reset communication with the USB device
    /// taken from https://github.com/commaai/openpilot/blob/master/selfdrive/boardd/panda.cc#L270
    void deviceReset();

    int getCommaAIDeviceName();

private:
    std::unique_ptr<CANAdapter> canAdapter_;
//    const uint32_t bus_offset;

//protected:
//  // for unit tests
//  uint8_t receive_buffer[RECV_SIZE + sizeof(can_header) + 64];
//  uint32_t receive_buffer_size = 0;
//
//  Panda(uint32_t bus_offset) : bus_offset(bus_offset) {}
//  void pack_can_buffer(const capnp::List<cereal::CanData>::Reader &can_data_list,
//                         std::function<void(uint8_t *, size_t)> write_func);
//  bool unpack_can_buffer(uint8_t *data, uint32_t &size, std::vector<can_frame> &out_vec);
//  uint8_t calculate_checksum(uint8_t *data, uint32_t len);
};

template <class CANAdapter>
CANClient<CANAdapter>::CANClient() {

}

template <class CANAdapter>
bool CANClient<CANAdapter>::isConnected() const {
    return canAdapter_.isConnected();
}

template <class CANAdapter>
bool CANClient<CANAdapter>::isCommHealthy() const {
    return canAdapter_.isCommHealthy();
}

} // namespace can
} // namespace nav
