#include "cli.h"

// -----------------------------
// Small helpers
// -----------------------------
static inline int32_t clamp_i32(int64_t v, int32_t lo, int32_t hi) {
  if (v < (int64_t)lo) return lo;
  if (v > (int64_t)hi) return hi;
  return (int32_t)v;
}
static inline uint32_t clamp_u32(int64_t v, uint32_t lo, uint32_t hi) {
  if (v < (int64_t)lo) return lo;
  if (v > (int64_t)hi) return hi;
  return (uint32_t)v;
}

// -----------------------------
// Begin / Help
// -----------------------------
void Cli::begin(Stream& io, InverterProtocol& inv, Esp32TinyUart& uart) {
  io_ = &io;
  inv_ = &inv;
  uart_ = &uart;

  printHelp();
  io_->print("\r\n> ");
  last_drawn_len_ = 0;
}

void Cli::printHelp() {
  if (!io_) return;
  io_->println("ESP32 GEA3 Inverter CLI");
  io_->println("Commands:");
  io_->println("  help");
  io_->println("  enable 0|1              (start/stop 500ms frames + 1s heartbeat)");
  io_->println("  send                    (send one frame immediately)");
  io_->println("  addr <src> <dst>         (hex or dec; dst=inverter address)");
  io_->println("  f023 <u8>               (set ERD F023 payload byte)");
  io_->println("  f213 <u8>               (set ERD F213 payload byte)");
  io_->println("  state <u8>              (F020.state)");
  io_->println("  profile <u8>            (F020.profile)");
  io_->println("  unk <u16>               (F020.unknown_u16)");
  io_->println("  accel <u16>             (F020.accel_u16)");
  io_->println("  speed <s16>             (F020.speed_s16)");
  io_->println("  param <0-35> <u16>       (set one of 36 params in F020)");
  io_->println("  ff                      (set F020 payload to all 0xFF and send once)");
  io_->println("  show                    (print config + latest telemetry)");
  io_->println("  sniff                   (print outgoing wire frames as hex; Ctrl+C exits)");
  io_->println("  drive                   (interactive: arrows, s=start/stop, d=dir, Ctrl+C exits->FF)");
  io_->println("Notes:");
  io_->println("  - Values accept 0x... hex or decimal.");
}

// -----------------------------
// History
// -----------------------------
void Cli::histResetNav() {
  hist_nav_ = -1;
  hist_scratch_[0] = 0;
}

const char* Cli::histGet(uint8_t idx_from_oldest) const {
  if (hist_count_ == 0) return nullptr;
  if (idx_from_oldest >= hist_count_) return nullptr;

  uint8_t oldest = (uint8_t)((hist_head_ + HIST_MAX - hist_count_) % HIST_MAX);
  uint8_t idx = (uint8_t)((oldest + idx_from_oldest) % HIST_MAX);
  return hist_[idx];
}

void Cli::histAdd(const char* s) {
  if (!s || !*s) return;

  // skip exact duplicate of last entry
  if (hist_count_ > 0) {
    uint8_t last_idx = (uint8_t)((hist_head_ + HIST_MAX - 1) % HIST_MAX);
    if (strncmp(hist_[last_idx], s, HIST_LINE_MAX) == 0) {
      histResetNav();
      return;
    }
  }

  strncpy(hist_[hist_head_], s, HIST_LINE_MAX - 1);
  hist_[hist_head_][HIST_LINE_MAX - 1] = 0;

  hist_head_ = (uint8_t)((hist_head_ + 1) % HIST_MAX);
  if (hist_count_ < HIST_MAX) hist_count_++;

  histResetNav();
}

void Cli::clearInputLine() {
  // Clear only what we previously drew to avoid wrapping.
  io_->print('\r');
  io_->print("> ");
  for (size_t i = 0; i < last_drawn_len_; i++) io_->print(' ');
  io_->print('\r');
  io_->print("> ");
}

void Cli::redrawInputLine() {
  clearInputLine();
  if (n_ > 0) io_->write((const uint8_t*)line_, n_);
  last_drawn_len_ = n_;
}

void Cli::histRecallUp() {
  if (hist_count_ == 0) return;

  if (hist_nav_ == -1) {
    strncpy(hist_scratch_, line_, HIST_LINE_MAX - 1);
    hist_scratch_[HIST_LINE_MAX - 1] = 0;
    hist_nav_ = (int16_t)(hist_count_ - 1); // newest
  } else if (hist_nav_ > 0) {
    hist_nav_--;
  }

  const char* cmd = histGet((uint8_t)hist_nav_);
  if (!cmd) return;

  strncpy(line_, cmd, CLI_LINE_MAX - 1);
  line_[CLI_LINE_MAX - 1] = 0;
  n_ = strnlen(line_, CLI_LINE_MAX - 1);
  redrawInputLine();
}

void Cli::histRecallDown() {
  if (hist_count_ == 0) return;
  if (hist_nav_ == -1) return;

  if (hist_nav_ < (int16_t)(hist_count_ - 1)) {
    hist_nav_++;
    const char* cmd = histGet((uint8_t)hist_nav_);
    if (!cmd) return;

    strncpy(line_, cmd, CLI_LINE_MAX - 1);
    line_[CLI_LINE_MAX - 1] = 0;
    n_ = strnlen(line_, CLI_LINE_MAX - 1);
    redrawInputLine();
  } else {
    // restore scratch
    histResetNav();
    strncpy(line_, hist_scratch_, CLI_LINE_MAX - 1);
    line_[CLI_LINE_MAX - 1] = 0;
    n_ = strnlen(line_, CLI_LINE_MAX - 1);
    redrawInputLine();
  }
}

// -----------------------------
// Parsing helpers
// -----------------------------
void Cli::trim(char* s) {
  if (!s) return;
  // leading
  while (*s && (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n')) s++;
  // copy down if needed
  // (simple approach: use memmove from first non-space)
  // But we get passed line_ which we can adjust in-place by shifting.
  // We'll do it with a temporary pointer logic:
  // This function is only called with a mutable buffer that begins at line_[0].
}

bool Cli::parseU32(const char* s, uint32_t& out) {
  if (!s) return false;
  while (*s == ' ' || *s == '\t') s++;
  if (!*s) return false;

  char* endp = nullptr;
  unsigned long v = strtoul(s, &endp, 0);
  if (endp == s) return false;
  out = (uint32_t)v;
  return true;
}

bool Cli::parseI32(const char* s, int32_t& out) {
  if (!s) return false;
  while (*s == ' ' || *s == '\t') s++;
  if (!*s) return false;

  char* endp = nullptr;
  long v = strtol(s, &endp, 0);
  if (endp == s) return false;
  out = (int32_t)v;
  return true;
}

// A better trim that actually compacts the buffer:
static void trim_in_place(char* s) {
  if (!s) return;

  // left trim
  char* p = s;
  while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
  if (p != s) memmove(s, p, strlen(p) + 1);

  // right trim
  size_t n = strlen(s);
  while (n > 0) {
    char c = s[n - 1];
    if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
      s[n - 1] = 0;
      n--;
    } else break;
  }
}

// -----------------------------
// Sniff mode
// -----------------------------
void Cli::printHexLine(const uint8_t* data, size_t n) {
  for (size_t i = 0; i < n; i++) {
    io_->printf("%02X", data[i]);
    if (i + 1 < n) io_->print(' ');
  }
  io_->print("\r\n");
}

void Cli::startSniff() {
  if (!uart_) return;
  sniffing_ = true;
  sniff_in_frame_ = false;
  sniff_frame_n_ = 0;

  uart_->setTxCapture(true);
  io_->println("sniff: capturing UART TX frames. Ctrl+C to stop.");
}

void Cli::stopSniff() {
  if (!uart_) return;
  uart_->setTxCapture(false);
  sniffing_ = false;
  sniff_in_frame_ = false;
  sniff_frame_n_ = 0;

  io_->print("\r\nsniff: stopped.\r\n> ");
  last_drawn_len_ = 0;
  n_ = 0;
  line_[0] = 0;
  histResetNav();
  esc_state_ = 0;
}

void Cli::runSniff() {
  if (!uart_) return;

  uint8_t b;
  while (uart_->popTxByte(b)) {
    if (!sniff_in_frame_) {
      if (b == 0xE2) {
        sniff_in_frame_ = true;
        sniff_frame_n_ = 0;
        sniff_frame_[sniff_frame_n_++] = b;
      }
      continue;
    }

    if (sniff_frame_n_ < SNIFF_FRAME_MAX) {
      sniff_frame_[sniff_frame_n_++] = b;
    } else {
      // overflow -> reset
      sniff_in_frame_ = false;
      sniff_frame_n_ = 0;
      continue;
    }

    if (b == 0xE3) {
      printHexLine(sniff_frame_, sniff_frame_n_);
      sniff_in_frame_ = false;
      sniff_frame_n_ = 0;
    }
  }
}

// -----------------------------
// Drive mode (2-line dashboard, in-place)
// -----------------------------
void Cli::drawDriveDashboard(bool first_draw) {
  if (!io_ || !inv_) return;

  // ANSI sequences (widely supported)
  const char* CLR = "\x1b[2K"; // clear entire line
  const char* UP2 = "\x1b[2A"; // move cursor up 2 lines

  if (!first_draw) {
    // Move back to start of the 2-line block
    io_->print(UP2);
  }

  // Line 1: Telemetry
  io_->print("\r");
  io_->print(CLR);

  auto t = inv_->telemetry();
  if (t.valid) {
    io_->printf("Temp: %.2f*F  Current: %u  DCBus_V: %uV",
                (double)t.temp_f_x100 / 100.0,
                (unsigned)t.stator_current,
                (unsigned)t.vbus);
  } else {
    io_->print("Temp: --  Current: --  DCBus_V: --");
  }
  io_->print("\r\n");

  // Line 2: Drive status
  io_->print("\r");
  io_->print(CLR);

  io_->printf("drive: %s | dir=%s | speed=%ld | accel=%lu",
              drive_running_ ? "RUN" : "STOP",
              (drive_dir_ > 0) ? "CW" : "CCW",
              (long)drive_speed_,
              (unsigned long)drive_accel_);
  io_->print("\r\n");
}

void Cli::applyDriveAndSend() {
  if (!inv_) return;

  inv_->setEnabled(true);

  // Inverter expects a run byte in F020: 0x02=stopped, 0x04=running
  inv_->setRun(drive_running_);

  // accel is always "commanded" in drive mode
  inv_->setAccel((uint16_t)clamp_u32((int64_t)drive_accel_, 0, 65535));

  // speed is commanded; when STOPPED we send 0xFFFF (sentinel) to match inverter expectation
  int32_t signed_speed;
  if (drive_running_) {
    signed_speed = (int32_t)drive_speed_ * (int32_t)drive_dir_;
    signed_speed = clamp_i32(signed_speed, -16000, 16000);
    inv_->setSpeedSigned((int16_t)signed_speed);
  } else {
    inv_->setSpeedSigned((int16_t)-1); // 0xFFFF
  }

  inv_->sendOnce();

  // Update dashboard after every send
  drawDriveDashboard(false);
}

void Cli::startDrive() {
  if (!io_ || !inv_) return;

  driving_ = true;
  drive_running_ = false;
  drive_dir_ = +1;
  drive_speed_ = 0;
  drive_accel_ = 1000;
  drive_last_speed_ = 0;
  esc_state_ = 0;

  io_->println("drive: active (arrows adjust, s=start/stop, d=dir, Ctrl+C exits->FF).");

  // First draw creates the 2-line block
  drawDriveDashboard(true);
  drive_dashboard_drawn_ = true;

  // Push initial stop frame + redraw
  applyDriveAndSend();
}

void Cli::stopDrive(bool restore_ff) {
  if (!io_ || !inv_) return;

  driving_ = false;
  esc_state_ = 0;
  drive_dashboard_drawn_ = false;

  if (restore_ff) {
    // Restore inverter-expected idle payload (stopped + FFs)
    inv_->setRun(false);
    inv_->setF020DefaultsStopped();
    inv_->sendOnce();
  }

  io_->print("\r\ndrive: exited.\r\n> ");
  last_drawn_len_ = 0;
  n_ = 0;
  line_[0] = 0;
  histResetNav();
}

void Cli::runDriveKey(int c) {
  // Parse ANSI arrows: ESC [ A/B/C/D
  if (esc_state_ == 0) {
    if (c == 0x1B) { esc_state_ = 1; return; }
  } else if (esc_state_ == 1) {
    if (c == '[') { esc_state_ = 2; return; }
    esc_state_ = 0;
  } else if (esc_state_ == 2) {
    esc_state_ = 0;

    if (c == 'A') { // Up: speed +
      drive_speed_ = clamp_i32((int64_t)drive_speed_ + SPEED_STEP, 0, 16000);
      drive_last_speed_ = drive_speed_;
      applyDriveAndSend();
      return;
    }
    if (c == 'B') { // Down: speed -
      drive_speed_ = clamp_i32((int64_t)drive_speed_ - SPEED_STEP, 0, 16000);
      drive_last_speed_ = drive_speed_;
      applyDriveAndSend();
      return;
    }
    if (c == 'C') { // Right: accel +
      drive_accel_ = clamp_u32((int64_t)drive_accel_ + ACCEL_STEP, 0, 65535);
      applyDriveAndSend();
      return;
    }
    if (c == 'D') { // Left: accel -
      drive_accel_ = clamp_u32((int64_t)drive_accel_ - ACCEL_STEP, 0, 65535);
      applyDriveAndSend();
      return;
    }
    return;
  }

  // Non-escape keys
  if (c == 's' || c == 'S') {
    drive_running_ = !drive_running_;
    if (drive_running_) {
      if (drive_speed_ == 0 && drive_last_speed_ > 0) drive_speed_ = drive_last_speed_;
    } else {
      drive_last_speed_ = drive_speed_;
    }
    applyDriveAndSend();
    return;
  }

  if (c == 'd' || c == 'D') {
    drive_dir_ = (drive_dir_ > 0) ? -1 : +1;
    applyDriveAndSend();
    return;
  }
}

// -----------------------------
// Command dispatch
// -----------------------------
void Cli::handleLine(const char* s) {
  if (!io_ || !inv_) return;
  if (!s || !*s) return;

  if (strcmp(s, "help") == 0) { printHelp(); return; }
  if (strcmp(s, "send") == 0) { inv_->sendOnce(); io_->println("sent"); return; }
  if (strcmp(s, "sniff") == 0) { startSniff(); return; }
  if (strcmp(s, "drive") == 0) { startDrive(); return; }
  if (strcmp(s, "ff") == 0) {
    inv_->setF020AllFF();
    inv_->sendOnce();
    io_->println("F020 set to all 0xFF and sent");
    return;
  }

  // Tokenize
  char buf[CLI_LINE_MAX];
  strncpy(buf, s, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = 0;

  char* save = nullptr;
  char* cmd = strtok_r(buf, " \t", &save);
  if (!cmd) return;

  if (strcmp(cmd, "enable") == 0) {
    char* a = strtok_r(nullptr, " \t", &save);
    uint32_t v;
    if (!a || !parseU32(a, v)) { io_->println("usage: enable 0|1"); return; }
    inv_->setEnabled(v ? true : false);
    io_->printf("enabled=%u\r\n", inv_->enabled() ? 1 : 0);
    return;
  }

  if (strcmp(cmd, "addr") == 0) {
    char* a = strtok_r(nullptr, " \t", &save);
    char* b = strtok_r(nullptr, " \t", &save);
    uint32_t src, dst;
    if (!a || !b || !parseU32(a, src) || !parseU32(b, dst)) {
      io_->println("usage: addr <src> <dst>");
      return;
    }
    inv_->setAddresses((uint8_t)src, (uint8_t)dst);
    io_->printf("addr set: src=0x%02X dst=0x%02X\r\n", (unsigned)inv_->src(), (unsigned)inv_->dst());
    return;
  }

  if (strcmp(cmd, "f023") == 0) {
    char* a = strtok_r(nullptr, " \t", &save);
    uint32_t v;
    if (!a || !parseU32(a, v)) { io_->println("usage: f023 <u8>"); return; }
    inv_->setF023((uint8_t)v);
    io_->printf("f023=0x%02X\r\n", (unsigned)(uint8_t)v);
    return;
  }

  if (strcmp(cmd, "f213") == 0) {
    char* a = strtok_r(nullptr, " \t", &save);
    uint32_t v;
    if (!a || !parseU32(a, v)) { io_->println("usage: f213 <u8>"); return; }
    inv_->setF213((uint8_t)v);
    io_->printf("f213=0x%02X\r\n", (unsigned)(uint8_t)v);
    return;
  }

  if (strcmp(cmd, "state") == 0) {
    char* a = strtok_r(nullptr, " \t", &save);
    uint32_t v;
    if (!a || !parseU32(a, v)) { io_->println("usage: state <u8>"); return; }
    inv_->setState((uint8_t)v);
    io_->printf("state=0x%02X\r\n", (unsigned)(uint8_t)v);
    return;
  }

  if (strcmp(cmd, "profile") == 0) {
    char* a = strtok_r(nullptr, " \t", &save);
    uint32_t v;
    if (!a || !parseU32(a, v)) { io_->println("usage: profile <u8>"); return; }
    inv_->setProfile((uint8_t)v);
    io_->printf("profile=0x%02X\r\n", (unsigned)(uint8_t)v);
    return;
  }

  if (strcmp(cmd, "unk") == 0) {
    char* a = strtok_r(nullptr, " \t", &save);
    uint32_t v;
    if (!a || !parseU32(a, v)) { io_->println("usage: unk <u16>"); return; }
    inv_->setUnknownU16((uint16_t)v);
    io_->printf("unk=0x%04X\r\n", (unsigned)(uint16_t)v);
    return;
  }

  if (strcmp(cmd, "accel") == 0) {
    char* a = strtok_r(nullptr, " \t", &save);
    uint32_t v;
    if (!a || !parseU32(a, v)) { io_->println("usage: accel <u16>"); return; }
    inv_->setAccel((uint16_t)v);
    io_->printf("accel=0x%04X\r\n", (unsigned)(uint16_t)v);
    return;
  }

  if (strcmp(cmd, "speed") == 0) {
    char* a = strtok_r(nullptr, " \t", &save);
    int32_t v;
    if (!a || !parseI32(a, v)) { io_->println("usage: speed <s16>"); return; }
    v = clamp_i32(v, -32768, 32767);
    inv_->setSpeedSigned((int16_t)v);
    io_->printf("speed=%ld\r\n", (long)v);
    return;
  }

  if (strcmp(cmd, "param") == 0) {
    char* a = strtok_r(nullptr, " \t", &save);
    char* b = strtok_r(nullptr, " \t", &save);
    uint32_t idx, val;
    if (!a || !b || !parseU32(a, idx) || !parseU32(b, val)) {
      io_->println("usage: param <0-35> <u16>");
      return;
    }
    if (!inv_->setParam((uint8_t)idx, (uint16_t)val)) {
      io_->println("param index out of range (0-35)");
      return;
    }
    io_->printf("param[%u]=0x%04X\r\n", (unsigned)idx, (unsigned)(uint16_t)val);
    return;
  }

  if (strcmp(cmd, "show") == 0) {
    auto t = inv_->telemetry();
    if (t.valid) {
      io_->printf("Telemetry: Temp=%.2f*F  Current=%u  Vbus=%uV\r\n",
                  (double)t.temp_f_x100 / 100.0,
                  (unsigned)t.stator_current,
                  (unsigned)t.vbus);
      io_->printf("F219 flags: %02X %02X %02X %02X %02X %02X\r\n",
                  t.f219_flags[0], t.f219_flags[1], t.f219_flags[2],
                  t.f219_flags[3], t.f219_flags[4], t.f219_flags[5]);
    } else {
      io_->println("Telemetry: (no data yet)");
    }
    io_->printf("Enabled: %u  Addr: src=0x%02X dst=0x%02X\r\n",
                inv_->enabled() ? 1 : 0,
                (unsigned)inv_->src(),
                (unsigned)inv_->dst());
    return;
  }

  io_->println("unknown command (type: help)");
}

// -----------------------------
// Main run loop
// -----------------------------
void Cli::run() {
  if (!io_ || !inv_) return;

  // If sniffing, keep draining TX capture continuously
  if (sniffing_) {
    runSniff();
  }

  // Drive mode: update dashboard on RX even without keyboard input,
  // and exit on comm-fault (no inverter responses).
  if (driving_) {
    if (inv_->consumeTelemetryUpdated()) {
      drawDriveDashboard(false);
    }
    // If inverter stops responding, bail out so you don't keep driving blind.
    if (inv_->msSinceLastRx() > 10000) {
      io_->print("\r\nERROR: Communication fault with inverter (no status RX). Exiting drive mode.\r\n");
      stopDrive(true /*restore_ff*/);
      return;
    }
  }

  while (io_->available()) {
    int c = io_->read();
    if (c < 0) break;

    // Ctrl+C cancels sniff/drive modes, or just resets prompt in normal mode
    if (c == 0x03) {
      if (driving_) {
        stopDrive(true /*restore_ff*/);
      } else if (sniffing_) {
        stopSniff();
      } else {
        io_->print("\r\n> ");
        last_drawn_len_ = 0;
        n_ = 0;
        line_[0] = 0;
        histResetNav();
        esc_state_ = 0;
      }
      continue;
    }

    // Drive mode consumes keystrokes (no scrolling)
    if (driving_) {
      runDriveKey(c);
      continue;
    }

    // Sniff mode ignores input except Ctrl+C
    if (sniffing_) {
      continue;
    }

    // -------- Normal CLI mode (history, in-place redraw) --------

    // Escape parsing for history arrows (ESC [ A/B)
    if (esc_state_ == 0) {
      if (c == 0x1B) { esc_state_ = 1; continue; }
    } else if (esc_state_ == 1) {
      if (c == '[') { esc_state_ = 2; continue; }
      esc_state_ = 0;
      // fall through
    } else if (esc_state_ == 2) {
      esc_state_ = 0;
      if (c == 'A') { // Up
        histRecallUp();
        continue;
      }
      if (c == 'B') { // Down
        histRecallDown();
        continue;
      }
      // ignore other sequences
      continue;
    }

    // Enter
    if (c == '\r' || c == '\n') {
      io_->print("\r\n");

      line_[n_] = 0;
      trim_in_place(line_);

      if (n_ > 0) {
        histAdd(line_);
        handleLine(line_);
      }

      // reset line
      n_ = 0;
      line_[0] = 0;
      last_drawn_len_ = 0;
      histResetNav();

      io_->print("> ");
      continue;
    }

    // Backspace/Delete
    if (c == 0x08 || c == 0x7F) {
      if (n_ > 0) {
        n_--;
        line_[n_] = 0;
        redrawInputLine();
      }
      histResetNav();
      continue;
    }

    // Ignore other control chars
    if (c < 0x20) continue;

    // Append character
    if (n_ < CLI_LINE_MAX - 1) {
      line_[n_++] = (char)c;
      line_[n_] = 0;
      histResetNav();
      redrawInputLine();
    }
  }
}