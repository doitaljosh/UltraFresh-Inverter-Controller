#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <string.h>

extern "C" {
#include "tiny_gea3_interface.h"
#include "tiny_gea_packet.h"
#include "tiny_event_subscription.h"
#include "hal/i_tiny_uart.h"
#include "tiny_timer.h" // for tiny_timer_group_t* signature compatibility
}

class InverterProtocol {
public:
  struct Telemetry {
    bool valid = false;
    uint8_t  f219_flags[6] = {0};
    uint16_t temp_f_x100 = 0;     // °F * 100
    uint16_t stator_current = 0;  // mA
    uint16_t vbus = 0;            // V
  };

  InverterProtocol();

  // Heartbeat LED indicator (toggles on each F026 increment)
  void enableHeartbeatLed(bool enable, int pin = 2, bool active_high = true);

  // Keep this signature to match main.cpp usage:
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

  // -----------------------------
  // F020 payload layout (0x58 bytes) - per your capture
  //
  //  [0] run/state byte: 0x02=stopped, 0x04=running
  //  [1] change counter: increments when ANY OTHER byte in F020 changes
  //  [2] FF
  //  [3] FF
  //  [4..5] accel u16 BE   (payload bytes 5-6, 1-based)
  //  [6..7] speed s16 BE   (payload bytes 7-8, 1-based)
  //  [8..0x57] FF (unless you explicitly write more)
  // -----------------------------

  // Legacy command names (keep CLI stable)
  void setState(uint8_t v)   { setRunByte(v); } // expects 0x02 or 0x04
  void setProfile(uint8_t)   {}                 // unused with the corrected mapping
  void setUnknownU16(uint16_t) {}               // unused with the corrected mapping

  void setRun(bool running);
  void setRunByte(uint8_t v);        // 0x02 or 0x04

  void setAccel(uint16_t accel_u16); // BE u16
  void setSpeedSigned(int16_t speed_s16); // BE s16

  // Param words starting at byte 8 (index 0 -> bytes 8..9), BE
  bool setParam(uint8_t index, uint16_t value);

  // Set any F020 byte except counter byte [1]. If changed, counter increments.
  bool setF020Byte(uint8_t index, uint8_t value);

  // Reset to inverter-expected "stopped defaults":
  // byte0=0x02, byte2..end=0xFF, accel/speed = 0xFFFF, counter preserved+incremented if anything changes
  void setF020DefaultsStopped();

  // Force all bytes to 0xFF (generally NOT recommended for byte0/byte1)
  void setF020AllFF();

  Telemetry telemetry() const { return telem_; }

  // Telemetry update tracking
  bool consumeTelemetryUpdated();
  uint32_t msSinceLastRx() const;

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

  // Heartbeat LED
  bool hb_led_enabled_ = false;
  int hb_led_pin_ = 2;
  bool hb_led_active_high_ = true;
  bool hb_led_state_ = false;
  void toggleHeartbeatLed_();

  tiny_gea3_interface_t gea3_{};
  tiny_event_subscription_t rx_sub_{};

  static constexpr size_t SEND_Q_SIZE = 512;
  static constexpr uint8_t RX_BUF_SIZE = 255; // tiny-gea-api build uses uint8_t length

  uint8_t send_q_[SEND_Q_SIZE]{};
  uint8_t rx_buf_[RX_BUF_SIZE]{};

  uint8_t src_addr_ = 0xC0;
  uint8_t dst_addr_ = 0x3F;

  bool enabled_ = false;
  uint32_t last_500ms_ms_ = 0;
  uint32_t last_1s_ms_ = 0;

  uint8_t f020_[F020_LEN]{};
  uint8_t f023_ = 0x00;
  uint8_t f026_ = 0x00;
  uint8_t f213_ = 0x00;

  Telemetry telem_{};

  // RX tracking
  volatile bool telem_dirty_ = false;
  uint32_t last_rx_ms_ = 0;

  // Packet events
  static void onPacketStatic(void* ctx, const void* args);
  void onPacket(const tiny_gea_interface_on_receive_args_t* rx);
  void handleB8(const tiny_gea_packet_t* pkt);

  // Send callback
  static void fillPacketStatic(void* ctx, tiny_gea_packet_t* pkt);
  void fillPacket(tiny_gea_packet_t* pkt);

  // F020 mutation helper: increments counter once if any byte except [1] changes
  void updateF020Range_(uint8_t index, const uint8_t* data, size_t len);

  static inline void put_u16_be(uint8_t* p, uint16_t v) {
    p[0] = (uint8_t)((v >> 8) & 0xFF);
    p[1] = (uint8_t)(v & 0xFF);
  }
  static inline uint16_t get_u16_be(const uint8_t* p) {
    return (uint16_t)((uint16_t)p[0] << 8) | (uint16_t)p[1];
  }
};
