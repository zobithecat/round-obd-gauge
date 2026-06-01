# Round OBD Gauge 🦞

A multi-functional **OBD-II BLE gauge** for the 1.28" round display board
(**ESP32-2424S012**, GC9A01 240×240 IPS + CST816D touch). Connects to an
ELM327-style BLE adapter and shows live car data as bold, animated round gauges.

Built on top of [LovyanGFX](https://github.com/lovyan03/LovyanGFX). Single-core
ESP32-C3, no PSRAM — everything runs off one 8-bit full-screen sprite.

![board: ESP32-2424S012](https://img.shields.io/badge/board-ESP32--2424S012-red) ![mcu: ESP32--C3](https://img.shields.io/badge/mcu-ESP32--C3-blue)

## Features

- **5 gauges** — RPM · Speed · Coolant · Boost · Timing — swipe to switch
- **Filling-arc style** with per-gauge color gradients (no needle), anti-aliased caps
- **Center-zero gauges** for boost & timing (0 at 12 o'clock, fills both ways)
- **Touch (CST816D)** — swipe = change gauge, single-tap = peak-hold, long-press = reset peaks
- **0–100 km/h launch timer** (제로백) — double-tap the Speed gauge, N-mode red theme
- **Fuel-grade detector** (LOW / REG / PREM) inferred from *knock-retard under load*,
  not raw timing advance
- **Overheat / redline / over-boost alerts** — pulsing red ring on any gauge
- **Boot ceremony** — OpenClaw logo scales in (anti-aliased zoom)
- **Demo face** — synthetic animated gauges after 60 s with no BLE
- **Robust BLE lifecycle** — fast connect, ~2.5 s disconnect detection, ~4 s auto-reconnect,
  values reset to 0 on disconnect, Kalman filters re-init for instant recovery
- ~44 FPS full-frame redraw, CAN-safe polling (≤10 req/s, request-response paced)

## Layout

```
round_obd_gauge/   gauge firmware (PlatformIO, env esp32c3)
mock_icar_pro/     fake "IOS-Vlink" OBD2 BLE adapter for bench testing (ESP32-S3)
```

## Build & flash

```bash
# gauge (ESP32-C3 round display)
pio run -d round_obd_gauge -e esp32c3 -t upload

# mock adapter — pick the env matching your S3 board:
pio run -d mock_icar_pro -e esp32s3       -t upload   # native-USB S3 (4MB)
pio run -d mock_icar_pro -e esp32s3_uart  -t upload   # CH340 UART S3 (16MB)
```

The gauge auto-connects to a BLE adapter advertising `IOS-Vlink`
(set `OBD_BLE_NAME` in `round_obd_gauge.ino`).

## Mock adapter commands (serial, 115200)

`1`/`2`/`3`/`4` = idle / driving / sport / overheat · `q`/`w`/`e` = low/regular/premium fuel ·
`s` = status · `h` = help

## Hardware notes

- Display GC9A01: SCLK=6 MOSI=7 DC=2 CS=10, backlight=GPIO3, `invert=true`
- Touch CST816D: SDA=4 SCL=5 INT=0 RST=1 (addr 0x15)
- 16-bit full-screen sprite won't allocate on the C3 (fragmented SRAM) — the canvas is 8-bit.

---
🤖 Built with [Claude Code](https://claude.com/claude-code)
