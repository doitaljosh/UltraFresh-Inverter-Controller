# UltraFresh-Inverter-Controller

ESP32-based UART controller for GE UltraFresh washer inverter boards using the GEA3 protocol.

This project provides:

- A working GEA3 protocol implementation using tiny-gea-api
- Real-time inverter telemetry parsing
- A UART CLI interface
- Interactive drive mode for speed and acceleration control using a keyboard
- Frame debug output
- Modular architecture for later LCD / encoder UI expansion

---

# Hardware Target

- ESP32 Dev Module (4MB flash)
- GE Washer Inverter Board (Renesas RA6M1 based)
- Isolated UART interface required between ESP32 and inverter

---

# Pin Mapping (ESP32 Dev Module)

| Function        | ESP32 Pin |
|-----------------|-----------|
| Inverter TX     | GPIO17    |
| Inverter RX     | GPIO16    |
| USB Serial CLI  | GPIO1 / GPIO3 |

Notes:
- Inverter UART runs at 230400 baud
- Use proper isolation between ESP32 and inverter UART
- Logic level is 3.3V

---

# Protocol Overview

GEA3 Frame Format:

[SOF=E2]
[dst]
[len]
[src]
[ERDCMD=0xB8]
[num_erds]
[erd_hi][erd_lo][len][payload]
...
[CRC16_hi][CRC16_lo]
[EOF=E3]

Special bytes:
0xE2 = SOF
0xE3 = EOF
0xE0 = Escape
0xE1 = ACK

---

# ERDs Used

Writes (ESP32 → Inverter):

F020 – Motor parameters block (0x58 bytes)  
F023 – Single byte control  
F026 – 1-byte heartbeat counter  
F213 – Reserved byte  

Telemetry (Inverter → ESP32):

F219 – 6-byte status flags  
F22E – IPM Temp (°F × 100) + Stator Current  
F24E – DC Bus Voltage  

---

# CLI Commands

General Commands:

help  
show  
addr src dst  
send  
enable 1  
enable 0  

ERD Configuration Commands:

f023 <u8>  
f213 <u8>  
state <u8>  
profile <u8>  
accel <u16>  
speed <s16>  
param <0-35> <u16>  
allff  

Values accept decimal or 0xHEX.

---

# Drive Mode

Enter:

drive

Keyboard Controls:

Up Arrow    → Increase speed (+500)  
Down Arrow  → Decrease speed (-500)  
Left Arrow  → Decrease acceleration (-500)  
Right Arrow → Increase acceleration (+500)  
s           → Start / Stop  
d           → Toggle direction (CW / CCW)  
Ctrl+C      → Exit drive mode and reset F020 to all 0xFF  

Example Output:

Temp: 102.45°F  Current: 18  DCBus_V: 326V
drive: RUN | dir=CW | speed=1500 | accel=2000

---

# Frame Debug Mode

Enter:

frames

Example Output:

E2 3F 5C 70 B8 04 F0 20 ...

Press Ctrl+C to exit.

---

# Default Initialization Behavior

On boot:
- All F020 bytes default to 0xFF
- Ramp and RPM are 0xFFFF
- Inverter expects either explicit values or full 0xFF payload

---

# Safety Notice

This project controls a 3-phase AC induction motor inverter.

Improper use can damage hardware or cause injury.

Always:
- Use isolation on UART
- Verify wiring
- Start with low speed
- Test without load first