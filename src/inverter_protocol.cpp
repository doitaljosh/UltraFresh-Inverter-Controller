#include "inverter_protocol.h"

InverterProtocol::InverterProtocol() {
  setF020AllFF();
  f023_ = 0xFF;
  f213_ = 0xFF;
  f026_ = 0x00;
}

void InverterProtocol::begin(tiny_timer_group_t* /*timers*/,
                            i_tiny_uart_t* uart,
                            uint8_t src_addr,
                            uint8_t dst_addr) {
  src_addr_ = src_addr;
  dst_addr_ = dst_addr;

  // Your installed tiny-gea-api DOES NOT take timers here.
  tiny_gea3_interface_init(
    &gea3_,
    uart,
    src_addr_,
    send_q_,
    sizeof(send_q_),
    rx_buf_,
    (uint8_t)sizeof(rx_buf_),
    false /*ignore_destination_address*/);

  tiny_event_subscription_init(&rx_sub_, this, &InverterProtocol::onPacketStatic);
  tiny_event_subscribe(tiny_gea_interface_on_receive(&gea3_.interface), &rx_sub_);

  last_500ms_ms_ = millis();
  last_1s_ms_ = millis();
}

void InverterProtocol::setAddresses(uint8_t src, uint8_t dst) {
  src_addr_ = src;
  dst_addr_ = dst;
  gea3_.address = src_addr_;
}

void InverterProtocol::setEnabled(bool en) {
  enabled_ = en;
  last_500ms_ms_ = millis();
  last_1s_ms_ = millis();
}

bool InverterProtocol::setParam(uint8_t index, uint16_t value) {
  if (index >= 36) return false;
  f020_.fields.params[index] = value;
  return true;
}

void InverterProtocol::setF020AllFF() {
  memset(f020_.raw, 0xFF, sizeof(f020_.raw));
}

void InverterProtocol::enableHeartbeatLed(bool enable, int pin, bool active_high) {
  hb_led_enabled_ = enable;
  hb_led_pin_ = pin;
  hb_led_active_high_ = active_high;

  if (hb_led_enabled_) {
    pinMode(hb_led_pin_, OUTPUT);
    hb_led_state_ = false;
    digitalWrite(hb_led_pin_, hb_led_active_high_ ? LOW : HIGH); // LED off initially
  }
}

void InverterProtocol::toggleHeartbeatLed_() {
  if (!hb_led_enabled_) return;

  hb_led_state_ = !hb_led_state_;
  const int level = hb_led_state_
                      ? (hb_led_active_high_ ? HIGH : LOW)
                      : (hb_led_active_high_ ? LOW : HIGH);
  digitalWrite(hb_led_pin_, level);
}

void InverterProtocol::run() {
  tiny_gea3_interface_run(&gea3_);

  const uint32_t now = millis();

  if (enabled_ && (uint32_t)(now - last_1s_ms_) >= 1000) {
    last_1s_ms_ = now;
    f026_++;
    toggleHeartbeatLed_();
  }

  if (enabled_ && (uint32_t)(now - last_500ms_ms_) >= 500) {
    last_500ms_ms_ = now;
    sendOnce();
  }
}

void InverterProtocol::sendOnce() {
  const uint8_t payload_len =
    2 +                      // cmd + count
    (2 + 1 + F020_LEN) +     // F020
    (2 + 1 + 1) +            // F023
    (2 + 1 + 1) +            // F026
    (2 + 1 + 1);             // F213

  tiny_gea_interface_send(
    &gea3_.interface,
    dst_addr_,
    payload_len,
    this,
    &InverterProtocol::fillPacketStatic);
}

void InverterProtocol::fillPacketStatic(void* ctx, tiny_gea_packet_t* pkt) {
  static_cast<InverterProtocol*>(ctx)->fillPacket(pkt);
}

void InverterProtocol::fillPacket(tiny_gea_packet_t* pkt) {
  uint8_t* p = pkt->payload;

  p[0] = ERDCMD_INVERTER;
  p[1] = 4;
  p += 2;

  // F020
  put_u16_be(p, ERD_F020); p += 2;
  *p++ = F020_LEN;
  memcpy(p, f020_.raw, F020_LEN);
  p += F020_LEN;

  // F023
  put_u16_be(p, ERD_F023); p += 2;
  *p++ = 1;
  *p++ = f023_;

  // F026
  put_u16_be(p, ERD_F026); p += 2;
  *p++ = 1;
  *p++ = f026_;

  // F213
  put_u16_be(p, ERD_F213); p += 2;
  *p++ = 1;
  *p++ = f213_;
}

void InverterProtocol::onPacketStatic(void* ctx, const void* args) {
  static_cast<InverterProtocol*>(ctx)->onPacket(
    static_cast<const tiny_gea_interface_on_receive_args_t*>(args));
}

void InverterProtocol::onPacket(const tiny_gea_interface_on_receive_args_t* rx) {
  if (!rx || !rx->packet) return;
  handleB8(rx->packet);
}

void InverterProtocol::handleB8(const tiny_gea_packet_t* pkt) {
  if (!pkt) return;
  if (pkt->payload_length < 2) return;

  const uint8_t* p = pkt->payload;
  if (p[0] != ERDCMD_INVERTER) return;

  const uint8_t erd_count = p[1];
  p += 2;

  for (uint8_t i = 0; i < erd_count; i++) {
    if ((size_t)(p - pkt->payload) + 3 > pkt->payload_length) return;

    uint16_t erd = get_u16_be(p); p += 2;
    uint8_t len = *p++;

    if ((size_t)(p - pkt->payload) + len > pkt->payload_length) return;

    if (erd == ERD_F219 && len == 6) {
      memcpy(telem_.f219_flags, p, 6);
      telem_.valid = true;
    } else if (erd == ERD_F22E && len >= 4) {
      telem_.temp_f_x100    = get_u16_be(p + 0);
      telem_.stator_current = get_u16_be(p + 2);
      telem_.valid = true;
    } else if (erd == ERD_F24E && len >= 2) {
      telem_.vbus = get_u16_be(p);
      telem_.valid = true;
    }

    p += len;
  }
}