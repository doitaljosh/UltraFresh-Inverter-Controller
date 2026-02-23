#pragma once
#include <Arduino.h>

extern "C" {
#include "hal/i_tiny_uart.h"
#include "tiny_event.h"
}

class Esp32TinyUart {
public:
  Esp32TinyUart(HardwareSerial& port, uint32_t baud, int8_t rxPin, int8_t txPin);

  i_tiny_uart_t* iface() { return &wrapper_.iface; }

  // call frequently from loop() to publish RX bytes + deferred TX-complete events
  void poll();

  // --- TX capture (wire bytes) ---
  void setTxCapture(bool en) { tx_capture_enabled_ = en; }
  bool txCaptureEnabled() const { return tx_capture_enabled_; }

  // Pop one captured byte. Returns true if a byte was returned.
  bool popTxByte(uint8_t& out);

private:
  HardwareSerial& port_;

  tiny_event_t on_send_complete_{};
  tiny_event_t on_receive_{};

  struct Wrapper {
    i_tiny_uart_t iface;      // MUST be first
    Esp32TinyUart* owner;
  } wrapper_{};

  // Defer send-complete to avoid recursion
  volatile bool send_complete_pending_ = false;

  // --- TX capture ring buffer ---
  static constexpr size_t TX_CAP_SZ = 2048;
  volatile bool tx_capture_enabled_ = false;
  uint8_t tx_cap_[TX_CAP_SZ]{};
  volatile uint16_t tx_head_ = 0;
  volatile uint16_t tx_tail_ = 0;

  void pushTxByte(uint8_t b);

  static void send_fn(i_tiny_uart_t* self, uint8_t byte);
  static i_tiny_event_t* on_send_complete_fn(i_tiny_uart_t* self);
  static i_tiny_event_t* on_receive_fn(i_tiny_uart_t* self);

  static const i_tiny_uart_api_t API_;
};