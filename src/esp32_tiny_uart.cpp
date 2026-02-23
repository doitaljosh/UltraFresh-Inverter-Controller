#include "esp32_tiny_uart.h"

const i_tiny_uart_api_t Esp32TinyUart::API_ = {
  &Esp32TinyUart::send_fn,
  &Esp32TinyUart::on_send_complete_fn,
  &Esp32TinyUart::on_receive_fn,
};

Esp32TinyUart::Esp32TinyUart(HardwareSerial& port, uint32_t baud, int8_t rxPin, int8_t txPin)
  : port_(port)
{
  port_.begin(baud, SERIAL_8N1, rxPin, txPin);

  tiny_event_init(&on_send_complete_);
  tiny_event_init(&on_receive_);

  wrapper_.iface.api = &API_;
  wrapper_.owner = this;
}

void Esp32TinyUart::pushTxByte(uint8_t b) {
  if (!tx_capture_enabled_) return;
  uint16_t next = uint16_t((tx_head_ + 1) % TX_CAP_SZ);
  if (next == tx_tail_) {
    // overflow: drop oldest
    tx_tail_ = uint16_t((tx_tail_ + 1) % TX_CAP_SZ);
  }
  tx_cap_[tx_head_] = b;
  tx_head_ = next;
}

bool Esp32TinyUart::popTxByte(uint8_t& out) {
  if (tx_tail_ == tx_head_) return false;
  out = tx_cap_[tx_tail_];
  tx_tail_ = uint16_t((tx_tail_ + 1) % TX_CAP_SZ);
  return true;
}

void Esp32TinyUart::poll() {
  // Publish RX bytes
  while (port_.available() > 0) {
    int v = port_.read();
    if (v < 0) break;

    tiny_uart_on_receive_args_t args;
    args.byte = static_cast<uint8_t>(v);
    tiny_event_publish(&on_receive_, &args);
  }

  // Publish deferred send-complete (ONE per poll) to avoid re-entrancy
  if (send_complete_pending_) {
    send_complete_pending_ = false;
    tiny_event_publish(&on_send_complete_, nullptr);
  }
}

void Esp32TinyUart::send_fn(i_tiny_uart_t* self, uint8_t byte) {
  auto* w = reinterpret_cast<Wrapper*>(self);
  Esp32TinyUart* u = w->owner;

  u->port_.write(byte);

  // capture actual wire byte
  u->pushTxByte(byte);

  // defer completion
  u->send_complete_pending_ = true;
}

i_tiny_event_t* Esp32TinyUart::on_send_complete_fn(i_tiny_uart_t* self) {
  auto* w = reinterpret_cast<Wrapper*>(self);
  return &w->owner->on_send_complete_.interface;
}

i_tiny_event_t* Esp32TinyUart::on_receive_fn(i_tiny_uart_t* self) {
  auto* w = reinterpret_cast<Wrapper*>(self);
  return &w->owner->on_receive_.interface;
}