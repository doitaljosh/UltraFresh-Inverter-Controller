#pragma once
#include <Arduino.h>
#include <stdint.h>
#include <string.h>

#include "inverter_protocol.h"
#include "esp32_tiny_uart.h"

class Cli {
public:
  void begin(Stream& io, InverterProtocol& inv, Esp32TinyUart& uart);
  void run();

private:
  Stream* io_ = nullptr;
  InverterProtocol* inv_ = nullptr;
  Esp32TinyUart* uart_ = nullptr;

  // -----------------------------
  // Normal CLI line editing
  // -----------------------------
  static constexpr size_t CLI_LINE_MAX = 160;
  char line_[CLI_LINE_MAX]{};
  size_t n_ = 0;

  // For ANSI escape parsing (ESC [ A/B/C/D etc.)
  uint8_t esc_state_ = 0; // 0=none, 1=ESC, 2=ESC '['

  // For in-place redraw without wrapping
  size_t last_drawn_len_ = 0;

  // -----------------------------
  // Command history (normal mode)
  // -----------------------------
  static constexpr size_t HIST_MAX = 20;
  static constexpr size_t HIST_LINE_MAX = CLI_LINE_MAX;

  char hist_[HIST_MAX][HIST_LINE_MAX]{};
  uint8_t hist_count_ = 0;     // <= HIST_MAX
  uint8_t hist_head_ = 0;      // next write index
  int16_t hist_nav_ = -1;      // -1 = not navigating, else 0..hist_count_-1
  char hist_scratch_[HIST_LINE_MAX]{}; // what user typed before Up/Down

  void histAdd(const char* s);
  const char* histGet(uint8_t idx_from_oldest) const;
  void histResetNav();
  void histRecallUp();
  void histRecallDown();

  void clearInputLine();
  void redrawInputLine();

  // -----------------------------
  // Modes
  // -----------------------------
  bool sniffing_ = false;

  bool driving_ = false;
  bool drive_running_ = false;
  int8_t drive_dir_ = +1; // +1=CW, -1=CCW
  int32_t drive_speed_ = 0;      // magnitude
  uint32_t drive_accel_ = 0;
  int32_t drive_last_speed_ = 0;
  bool drive_dashboard_drawn_ = false;

  static constexpr int32_t SPEED_STEP = 500;
  static constexpr uint32_t ACCEL_STEP = 500;

  // For sniff framing
  bool sniff_in_frame_ = false;
  static constexpr size_t SNIFF_FRAME_MAX = 512;
  uint8_t sniff_frame_[SNIFF_FRAME_MAX]{};
  size_t sniff_frame_n_ = 0;

  // -----------------------------
  // Command handlers
  // -----------------------------
  void printHelp();
  void handleLine(const char* s);

  // sniff
  void startSniff();
  void stopSniff();
  void runSniff();
  void printHexLine(const uint8_t* data, size_t n);

  // drive
  void startDrive();
  void stopDrive(bool restore_ff);
  void runDriveKey(int c);
  void applyDriveAndSend();         // write accel/speed and sendOnce
  void drawDriveDashboard(bool first_draw);

  // -----------------------------
  // Parsing helpers
  // -----------------------------
  static void trim(char* s);
  static bool parseU32(const char* s, uint32_t& out);
  static bool parseI32(const char* s, int32_t& out);
};