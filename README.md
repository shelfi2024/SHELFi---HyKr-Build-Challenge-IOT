# SHELFi — Smart Shelf Hardware

Battery-backed, LoRaWAN-connected smart-shelf node for retail product monitoring. Reports calibrated product weight to the cloud whenever a meaningful inventory change is detected — designed for low-power, store-scale deployment.

> Part of [SHELFi](https://shelfi.example) — a smart in-shelf product monitoring platform for brands, suppliers, store managers, and warehouse teams.

---

## Architecture at a glance

Two-board design: a permanently-installed **Transmitter** under the shelf, and a detachable **Calibrator** handheld for setup and field diagnostics.

```
┌────────────────────────────────────────────────────────────────────────┐
│  TRANSMITTER (always-on, sealed)                                       │
│                                                                        │
│  Load Cell ─▶ HX710B (24-bit ΔΣ ADC) ─▶ STM32F103 ─▶ 31-tap FIR        │
│                                              │                         │
│                                              ▼                         │
│                                       Tare/Span scaling                │
│                                              │                         │
│                                              ▼                         │
│                          Event detector  (Δ ≥ 50 g, stable 3 s)        │
│                                              │                         │
│                                              ▼                         │
│                          RA-08H 915 MHz LoRaWAN  ──▶  Gateway / Cloud  │
└────────────────────────────────────────────────────────────────────────┘
                              ▲    │  hot-plug
                              │    ▼  (via 8-pin connector)
┌────────────────────────────────────────────────────────────────────────┐
│  CALIBRATOR (detachable, handheld)                                     │
│  16×2 I²C LCD  ·  3 tactile buttons  ·  no MCU — passive bridge        │
└────────────────────────────────────────────────────────────────────────┘
```

## Hardware

| Subsystem | Implementation |
|---|---|
| **MCU** | STM32F103C8T6 (Cortex-M3, 64 KB Flash, 20 KB SRAM) |
| **Sensing** | 1× or 2× HX710B 24-bit Σ-Δ ADC, bit-banged on PA4/PA5 (and PA9/PA10 in dual-cell variant) |
| **Radio** | RA-08H 915 MHz LoRaWAN module, USART2 @ 9600 baud, ABP mode with confirmed uplinks |
| **Power** | 12 V mains + 12 V battery backup (11.1 – 12.4 V), AP64100SP buck → 5 V, three independent RT9080-33 LDOs for MCU / load-cell front-end / LoRa |
| **Persistence** | STM32 emulated EEPROM stores tare and span values across power cycles |
| **User I/O** | Detachable calibrator with 16×2 LCD via PCF8574 I²C expander, 3 tactile buttons, hot-plug detected via LCD_FB feedback line |
| **Power gating** | LOADCELL_EN (PB12) and LORA_EN (PB13) allow the MCU to selectively power-cycle each subsystem |
| **Programming** | SWD via 2×4 header (also exposes UART for RA-08H AT commands) |

### Connectors (transmitter)

1. 12 V power input
2. 12 V battery input
3. Calibrator connector (8-pin: 5 V, GND, level-shifted I²C, 3 button lines)
4. LoRa UART & SWD programming (2×4)
5. Load cell input (5-pin: SIG+/SIG−, EXE+, AGND)

### Calibrator board

- Outer dimensions **56.134 × 85.344 mm**
- Draws power and I²C from transmitter; **no MCU of its own**
- HS1602A 16×2 char LCD via PCF8574AM/TR (I²C addr `0x27`)
- Three TS1243TP-SZ tactile switches: **TARE / SPAN / SAVE** (or **SWITCHER** in dual-cell firmware)
- Bidirectional I²C level translator (3.3 V transmitter ↔ 5 V calibrator) using two N-FETs

## Firmware

Two variants share one code-base style:

| File | Purpose |
|---|---|
| `Shelfi_IOT_LoRa.ino` | **Production** single load cell, device ID 11 |
| `Shelfi_Lora_2loadcell.ino` | **Dev** dual load cell, device IDs 31 / 32, channel-switcher button |
| `filter.c` | 31-tap symmetric FIR low-pass (Butterworth-style), 4 parallel instances |

### Signal pipeline

1. **Sample** — `readHX710B()` bit-bangs 24 bits MSB-first, sign-extends to 32-bit signed.
2. **Filter** — 31-tap symmetric FIR (`fir1`...`fir4`) with linear phase. Suppresses 50/60 Hz mains pickup and mechanical jitter without distorting step response. Output is right-shifted (`>> 1` single-cell, `>> 5` dual-cell) to fit int16 payload.
3. **Calibrate** — Two-point linear fit: `cal_factor = 1000.0 / (span - tare)`, then quantize to nearest `RES` grams (5 g single-cell, 10 g dual-cell). Handles either load-cell polarity.
4. **Detect** — Two-stage event detector:
   - **Trigger:** weight change ≥ `WEIGHT_DELTA` (50 g) vs last reported.
   - **Stability gate:** new weight must hold within ±50 g for `STABLE_TIME_MS` (3000 ms). Any movement restarts the timer — filters out customer hover.
5. **Transmit** — On commit, opens a 60-second TX window. `transmitCompactData()` fires every 400 ms inside the window (belt-and-braces against gateway congestion), then closes until the next event.

### LoRaWAN payload (8 bytes, big-endian)

| Bytes | Field | Notes |
|---|---|---|
| 0–1 | Device ID (uint16) | 11 (single-cell) / 31, 32 (dual-cell) |
| 2–3 | Filtered ADC (int16) | Post-FIR raw ADC for back-end signal-quality tracking |
| 4 | Battery percent (uint8) | 0–100, mapped from ADC 615 (9 V) → 737 (12 V) |
| 5 | Reserved (0x00) | Future flags / payload version |
| 6–7 | Weight in grams (int16) | Calibrated, quantized |

Sent as ASCII-hex via `AT+DTRX=1,2,8,<hex>` to the RA-08H. Module is configured in **ABP mode** with a fixed `DevAddr`, confirmed uplinks, DR5, ADR off — optimized for fast factory provisioning and predictable airtime.

### Calibration workflow

1. Plug calibrator into transmitter (LCD comes alive automatically — `LCD_FB` low triggers `initLCD()`).
2. Empty shelf → press **TARE** → zero point captured, written to EEPROM.
3. Place known reference mass → press **SPAN** → scale calibrated, written to EEPROM.
4. (Dual-cell only) Press **SWITCHER** to toggle between channel 1 and channel 2 calibration.
5. Unplug calibrator. Values persist across power cycles.

## Repository layout

```
hardware/
  Schematic_Smart_Shelf_Lora.pdf        # Transmitter schematic, Sheet_1 Rev 1.0
  Schematic_Smart_Self_Calibrater.pdf   # Calibrator schematic, Sheet_1 Rev 1.0
  Shelfi_transmitter_board_specifications.docx
  Shelfi_calibrator_board_specifications.docx
firmware/
  Shelfi_IOT_LoRa.ino                   # Production single-load-cell build
  Shelfi_Lora_2loadcell.ino             # Dev dual-load-cell build
  filter.c                              # 31-tap FIR coefficients + filter functions
  build/
    Shelfi_IOT_LoRa_ino.bin             # ~45 KB compiled binary
    Shelfi_IOT_LoRa_ino.hex             # Intel HEX for flashing
    Shelfi_IOT_LoRa_ino.elf             # ELF with debug symbols
    Shelfi_IOT_LoRa_ino.map             # Linker map
docs/
  SHELFi_Hardware_Technical_Report.pdf  # Full technical report
```

## Build & flash

Built with the **Arduino IDE + STM32 core** (Arduino_Core_STM32). Required libraries: `Wire`, `LiquidCrystal_I2C`, `EEPROM`, `HardwareSerial`. Flash via the SWD header on the 2×4 connector using ST-Link or any compatible CMSIS-DAP probe.

```bash
# Example using arduino-cli (board: STM32F1, variant: BluePill F103C8)
arduino-cli compile --fqbn STMicroelectronics:stm32:GenF1:pnum=BLUEPILL_F103C8 firmware/Shelfi_IOT_LoRa.ino
arduino-cli upload --fqbn STMicroelectronics:stm32:GenF1:pnum=BLUEPILL_F103C8 -p <port> firmware/Shelfi_IOT_LoRa.ino
```

## Key design decisions

- **Two-board split** — sealed, cheap shelf nodes; one calibrator services many. Keeps RF/EMI clean during normal operation.
- **HX710B over HX711** — built-in temperature sensor, on-chip oscillator, tighter noise floor for low-amplitude shelf-weight changes.
- **Symmetric 31-tap FIR** — linear-phase low-pass means the step response stays sharp; critical for accurate "item placed" detection.
- **50 g / 3 s gate** — empirically tuned to ignore micro-vibration and customer hover while catching every real SKU placement / removal.
- **Per-rail LDO gating** — load cell and LoRa rails are independently switchable, opening the door for deep-sleep modes without a hardware respin.
- **Reserved payload byte** — byte 5 is currently `0x00`; reserved for a future payload-version + flags field (low-battery, motion-detected, calibrated/uncalibrated).

## Known limitations & roadmap

- Static device IDs hard-coded in firmware → migrate to EEPROM-stored, calibrator-settable IDs.
- No MCU sleep — STM32 stop mode + RTC wake would extend battery life significantly.
- 60 s × 400 ms uplink burst is generous; consider 3–5 confirmed packets with exponential back-off.
- No downlink command parser yet (remote tare, remote ID, OTA trigger).
- FIR output shifts (`>> 1`, `>> 5`) hard-coded; should be a single configurable scale factor.


## License

Proprietary. All rights reserved.
