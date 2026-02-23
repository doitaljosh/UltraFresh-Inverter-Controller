#include "inverter_protocol.h"

InverterProtocol::InverterProtocol() {
  // Defaults: stopped + FFs
  memset(f020_, 0xFF, sizeof(f020_));
  f020_[0] = 0x02; // stopped
  f020_[1] = 0x00; // change counter

  f023_ = 0x00;
  f213_ = 0x02;
  f026_ = 0x00;
}

void InverterProtocol::enableHeartbeatLed(bool enable, int pin, bool active_high) {
  hb_led_enabled_ = enable;
  hb_led_pin_ = pin;
  hb_led_active_high_ = active_high;

  if (hb_led_enabled_) {
    pinMode(hb_led_pin_, OUTPUT);
    hb_led_state_ = false;
    digitalWrite(hb_led_pin_, hb_led_active_high_ ? LOW : HIGH);
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

void InverterProtocol::begin(tiny_timer_group_t* /*timers*/,
                            i_tiny_uart_t* uart,
                            uint8_t src_addr,
                            uint8_t dst_addr) {
  src_addr_ = src_addr;
  dst_addr_ = dst_addr;

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
  last_rx_ms_ = millis();
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

bool InverterProtocol::consumeTelemetryUpdated() {
  if (!telem_dirty_) return false;
  telem_dirty_ = false;
  return true;
}

uint32_t InverterProtocol::msSinceLastRx() const {
  return (uint32_t)(millis() - last_rx_ms_);
}

// -----------------------------
// F020 mutation helper
// -----------------------------
void InverterProtocol::updateF020Range_(uint8_t index, const uint8_t* data, size_t len) {
  if (!data || len == 0) return;
  if (index >= F020_LEN) return;
  if ((size_t)index + len > F020_LEN) return;

  bool changed = false;
  for (size_t i = 0; i < len; i++) {
    uint8_t idx = (uint8_t)(index + i);
    if (idx == 1) continue; // ignore counter
    if (f020_[idx] != data[i]) { changed = true; break; }
  }
  if (!changed) return;

  // bump counter once per logical change
  f020_[1] = (uint8_t)(f020_[1] + 1);

  for (size_t i = 0; i < len; i++) {
    uint8_t idx = (uint8_t)(index + i);
    if (idx == 1) continue;
    f020_[idx] = data[i];
  }
}

void InverterProtocol::setRun(bool running) {
  setRunByte(running ? 0x04 : 0x02);
}

void InverterProtocol::setRunByte(uint8_t v) {
  if (v != 0x02 && v != 0x04) return;
  updateF020Range_(0, &v, 1);
}

void InverterProtocol::setAccel(uint16_t accel_u16) {
  uint8_t b[2];
  put_u16_be(b, accel_u16);
  updateF020Range_(4, b, 2);
}

void InverterProtocol::setSpeedSigned(int16_t speed_s16) {
  uint8_t b[2];
  put_u16_be(b, (uint16_t)speed_s16);
  updateF020Range_(6, b, 2);
}

bool InverterProtocol::setParam(uint8_t index, uint16_t value) {
  // Params are u16 words starting at byte 8
  if (index >= 36) return false;
  uint8_t b[2];
  put_u16_be(b, value);
  uint8_t off = (uint8_t)(8 + (index * 2));
  updateF020Range_(off, b, 2);
  return true;
}

bool InverterProtocol::setF020Byte(uint8_t index, uint8_t value) {
  if (index >= F020_LEN) return false;
  if (index == 1) return false;
  updateF020Range_(index, &value, 1);
  return true;
}

void InverterProtocol::setF020DefaultsStopped() {
  // target: byte0=0x02, byte2..end=0xFF, accel/speed implicitly 0xFFFF
  uint8_t target[F020_LEN];
  memset(target, 0xFF, sizeof(target));
  target[0] = 0x02;
  // target[1] ignored

  bool diff = false;
  for (size_t i = 0; i < F020_LEN; i++) {
    if (i == 1) continue;
    if (f020_[i] != target[i]) { diff = true; break; }
  }
  if (!diff) return;

  // bump counter once
  f020_[1] = (uint8_t)(f020_[1] + 1);
  // apply
  for (size_t i = 0; i < F020_LEN; i++) {
    if (i == 1) continue;
    f020_[i] = target[i];
  }
}

void InverterProtocol::setF020AllFF() {
  bool diff = false;
  for (size_t i = 0; i < F020_LEN; i++) {
    if (i == 1) continue;
    if (f020_[i] != 0xFF) { diff = true; break; }
  }
  if (!diff) return;

  f020_[1] = (uint8_t)(f020_[1] + 1);
  for (size_t i = 0; i < F020_LEN; i++) {
    if (i == 1) continue;
    f020_[i] = 0xFF;
  }
}

// -----------------------------
// Main run / send / parse
// -----------------------------
void InverterProtocol::run() {
  tiny_gea3_interface_run(&gea3_);

  const uint32_t now = millis();

  // Heartbeat reflects link alive
  if ((uint32_t)(now - last_1s_ms_) >= 1000) {
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
  memcpy(p, f020_, F020_LEN);
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
  last_rx_ms_ = millis();
  telem_dirty_ = true;
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
