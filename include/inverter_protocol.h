#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <string.h>

extern "C" {
#include "tiny_gea3_interface.h"
#include "tiny_gea_packet.h"
#include "tiny_event_subscription.h"
#include "hal/i_tiny_uart.h"

// Only needed because your main.cpp passes a tiny_timer_group_t*.
// We accept it but do not use it (your tiny-gea-api build doesn't need it).
#include "tiny_timer.h"
}

class InverterProtocol {
public:
  struct Telemetry {
    bool valid = false;
    uint8_t  f219_flags[6] = {0};
    uint16_t temp_f_x100 = 0;     // °F * 100
    uint16_t stator_current = 0;  // raw units
    uint16_t vbus = 0;            // V
  };

  InverterProtocol();

  // Heartbeat LED indicator (toggles on each F026 increment)
  void enableHeartbeatLed(bool enable, int pin = 2, bool active_high = true);

  // Keep this signature to match your existing main.cpp call:
  // inverter.begin(&timers, inv_uart_adapter->iface(), SRC_ADDR, DST_ADDR);
  void begin(tiny_timer_group_t* timers, i_tiny_uart_t* uart, uint8_t src_addr, uint8_t dst_addr);

  void run();

  void setEnabled(bool en);
  bool enabled() const { return enabled_; }

  void sendOnce();

  void setAddresses(uint8_t src, uint8_t dst);
  uint8_t src() const { return src_addr_; }
  uint8_t dst() const { return dst_addr_; }

  void setF023(uint8_t v) { f023_ = v; }
  void setF213(uint8_t v) { f213_ = v; }

  void setState(uint8_t v)        { f020_.fields.state = v; }
  void setProfile(uint8_t v)      { f020_.fields.profile = v; }
  void setUnknownU16(uint16_t v)  { f020_.fields.unknown_u16 = v; }
  void setAccel(uint16_t v)       { f020_.fields.accel_u16 = v; }
  void setSpeedSigned(int16_t v)  { f020_.fields.speed_s16 = v; }

  bool setParam(uint8_t index, uint16_t value);

  void setF020AllFF();

  Telemetry telemetry() const { return telem_; }

private:
  static constexpr uint8_t ERDCMD_INVERTER = 0xB8;

  static constexpr uint16_t ERD_F020 = 0xF020;
  static constexpr uint16_t ERD_F023 = 0xF023;
  static constexpr uint16_t ERD_F026 = 0xF026;
  static constexpr uint16_t ERD_F213 = 0xF213;

  static constexpr uint16_t ERD_F219 = 0xF219;
  static constexpr uint16_t ERD_F22E = 0xF22E;
  static constexpr uint16_t ERD_F24E = 0xF24E;

  static constexpr uint8_t F020_LEN = 0x58;

  struct __attribute__((packed)) F020Fields {
    uint8_t  state;
    uint8_t  profile;
    uint16_t unknown_u16;
    uint16_t accel_u16;
    int16_t  speed_s16;
    uint16_t params[36];
    uint16_t reserved[4];
  };

  union F020Payload {
    F020Fields fields;
    uint8_t raw[F020_LEN];
  };

  bool hb_led_enabled_ = false;
  int hb_led_pin_ = 2;
  bool hb_led_active_high_ = true;
  bool hb_led_state_ = false;

void toggleHeartbeatLed_();

  tiny_gea3_interface_t gea3_{};
  tiny_event_subscription_t rx_sub_{};

  static constexpr size_t SEND_Q_SIZE = 512;

  // IMPORTANT: rx length argument is uint8_t in your tiny-gea-api build => must be <=255
  static constexpr uint8_t RX_BUF_SIZE = 255;

  uint8_t send_q_[SEND_Q_SIZE]{};
  uint8_t rx_buf_[RX_BUF_SIZE]{};

  uint8_t src_addr_ = 0x70;
  uint8_t dst_addr_ = 0x3F;

  bool enabled_ = false;
  uint32_t last_500ms_ms_ = 0;
  uint32_t last_1s_ms_ = 0;

  F020Payload f020_{};
  uint8_t f023_ = 0xFF;
  uint8_t f026_ = 0x00;
  uint8_t f213_ = 0xFF;

  Telemetry telem_{};

  static void onPacketStatic(void* ctx, const void* args);
  void onPacket(const tiny_gea_interface_on_receive_args_t* rx);

  void handleB8(const tiny_gea_packet_t* pkt);

  static void fillPacketStatic(void* ctx, tiny_gea_packet_t* pkt);
  void fillPacket(tiny_gea_packet_t* pkt);

  static inline void put_u16_be(uint8_t* p, uint16_t v) {
    p[0] = (uint8_t)((v >> 8) & 0xFF);
    p[1] = (uint8_t)(v & 0xFF);
  }
  static inline uint16_t get_u16_be(const uint8_t* p) {
    return (uint16_t)((uint16_t)p[0] << 8) | (uint16_t)p[1];
  }
};