#include <Arduino.h>

extern "C" {
#include "tiny_time_source.h"
#include "tiny_timer.h"
}

#include "esp32_tiny_uart.h"
#include "inverter_protocol.h"
#include "cli.h"

// Timers
static tiny_timer_group_t timers;
static i_tiny_time_source_t* time_source = nullptr;

// ESP32 UART to inverter
static Esp32TinyUart* inv_uart_adapter = nullptr;

// Protocol + CLI
static InverterProtocol inverter;
static Cli cli;

static constexpr uint32_t USB_BAUD = 115200;
static constexpr uint32_t INV_BAUD = 230400;

// Choose pins for your wiring (change these!)
static constexpr int8_t INV_RX_PIN = 16;
static constexpr int8_t INV_TX_PIN = 17;

// Addresses (change in CLI via addr command)
static constexpr uint8_t SRC_ADDR = 0xC0; // ESP host
static constexpr uint8_t DST_ADDR = 0x3F; // inverter

void setup() {
  Serial.begin(USB_BAUD);
  while (!Serial) { delay(10); }

  Serial.println("\nBooting...");

  time_source = tiny_time_source_init();
  tiny_timer_group_init(&timers, time_source);

  // Create UART adapter using Serial1 with explicit pins
  inv_uart_adapter = new Esp32TinyUart(Serial1, INV_BAUD, INV_RX_PIN, INV_TX_PIN);

  inverter.begin(&timers, inv_uart_adapter->iface(), SRC_ADDR, DST_ADDR);

  inverter.enableHeartbeatLed(true, 2, true);

  cli.begin(Serial, inverter, *inv_uart_adapter);

  Serial.println("Ready.");
}

void loop() {
  tiny_timer_group_run(&timers);
  inv_uart_adapter->poll();
  inverter.run();
  cli.run();
}