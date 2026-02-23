#pragma once
#include <stdint.h>

static inline uint16_t be16_read(const uint8_t* p) {
  return (uint16_t(p[0]) << 8) | uint16_t(p[1]);
}

static inline void be16_write(uint8_t* p, uint16_t v) {
  p[0] = uint8_t((v >> 8) & 0xFF);
  p[1] = uint8_t(v & 0xFF);
}

static inline void be16_write_s(uint8_t* p, int16_t v) {
  be16_write(p, (uint16_t)v);
}