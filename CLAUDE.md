# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Meshtastic firmware — open-source LoRa mesh networking for long-range, low-power communication without internet or cellular infrastructure. Supports text messaging, GPS position sharing, and sensor telemetry over a decentralized mesh.

**Active development target:** `xiao_nrf52840_almemo_sensor_receiver` (Seeed XIAO nRF52840 with I2C slave interface, set as default env in `platformio.ini`).

## Build Commands

```bash
# Build the default environment (xiao_nrf52840_almemo_sensor_receiver)
pio run

# Build a specific target
pio run -e tbeam

# Build and upload to device
pio run -e tbeam -t upload

# Flash nRF52 via DFU over serial (device-specific)
python ~/.platformio/packages/tool-adafruit-nrfutil/adafruit-nrfutil.py dfu serial \
  --package .pio/build/xiao_nrf52840_almemo_sensor_receiver/firmware-xiao_nrf52840_almemo_sensor_receiver-*.zip \
  -p /dev/ttyACM0 -b 115200 --singlebank --touch 1200

# Build native Linux version
pio run -e native

# Run unit tests (native platform only)
pio test -e native

# Format code before committing
trunk fmt

# Regenerate protobuf code
bin/regen-protos.sh
```

## Architecture Overview

### Threading Model

FreeRTOS-based on embedded targets. Background tasks inherit from `OSThread` (in `src/concurrency/`). Periodic tasks return the next run interval in ms from `runOnce()`. Radio SPI access is protected by `SPILock`; general mutex use via `concurrency::Lock`.

### Module System

Feature modules inherit from `MeshModule` or `ProtobufModule<T>` and are registered in `src/modules/Modules.cpp`. Key methods to implement:

- `handleReceivedProtobuf()` — process incoming packets
- `allocReply()` — generate response packets
- `runOnce()` — periodic task (returns next interval in ms)

### Radio / Mesh Stack

- `src/mesh/RadioInterface.*` — abstract radio base class
- `src/mesh/RadioLibInterface.*` — RadioLib-based implementations (SX126x, SX128x, LR11x0)
- `src/mesh/Router.*` / `FloodingRouter.*` / `ReliableRouter.*` — routing strategies
- `src/mesh/NodeDB.*` — persistent node database
- `src/mesh/Channels.*` — channel encryption and config
- `src/mesh/MeshService.*` — high-level mesh API exposed to phone/app via `PhoneAPI`

### Configuration Access

- `config.*` — device config (LoRa, position, power, display, …)
- `moduleConfig.*` — per-module config
- `channels.*` — channel config and management
- Default helpers in `src/mesh/Default.h`: `getConfiguredOrDefaultMs()`, `getConfiguredOrMinimumValue()`, `getConfiguredOrDefaultMsScaled()`

### Platform Abstraction

Platform-specific HAL code lives in `src/platform/<arch>/`. Hardware capability and pin definitions per board are in `variants/<arch>/<name>/variant.h`.

### Display / Input

- `src/graphics/Screen.*` — display abstraction; drivers for OLED, TFT, e-ink under `src/graphics/`
- `src/input/InputBroker.*` — routes events from buttons, keyboards, rotary encoders, touchscreens

### Custom I2C Slave (local extension)

`src/custom/` contains a local I2C slave integration (not upstreamed). It wraps the nRF52 TWI peripheral as a slave so the XIAO can be commanded by an external host over I2C.

## Hardware Variants

Each variant directory (`variants/<arch>/<name>/`) contains:
- `variant.h` — pin defines and `#define USE_SX126x` / `HAS_GPS` / `HAS_SCREEN` etc.
- `platformio.ini` — build config (`extends`, `build_flags`, `board_level`)

`board_level` controls CI inclusion:
- `pr` — built on every PR
- *(default)* — built on merges to main branches
- `extra` — release builds only

The local variant hierarchy for this repo:

```
seeed_xiao_nrf52840_kit          (GPS on D6/D7, I2C off)
  └─ seeed_xiao_nrf52840_kit_i2c (GPS on NFC pins, I2C on D6/D7)
       └─ xiao_nrf52840_almemo_sensor_receiver (+ EXCLUDE_I2C + I2C slave on raw pins 43/44)
```

## Coding Conventions

- Naming: `PascalCase` classes, `camelCase` functions/members, `UPPER_SNAKE_CASE` constants/defines, `USERPREFS_*` for user-configurable options
- Logging: `LOG_DEBUG`, `LOG_INFO`, `LOG_WARN`, `LOG_ERROR`
- Feature guards: `#if !MESHTASTIC_EXCLUDE_GPS`, `#ifdef ARCH_ESP32`, `#if defined(USE_SX1262)`, `#ifdef HAS_SCREEN`
- Run `trunk fmt` before committing (CI enforces via `trunk_check.yml`)

## Protobuf Messages

Defined in `protobufs/meshtastic/*.proto`, generated into `src/mesh/generated/`. All types are prefixed `meshtastic_`. Regenerate with `bin/regen-protos.sh`.

## Important Constraints

**Traffic management** — mesh bandwidth is limited. Always use `Default::getConfiguredOrMinimumValue()` for broadcast intervals, and scale with `numOnlineNodes`.

**Power management** — many devices are battery-powered. Use `IF_ROUTER(routerVal, normalVal)` for role-based defaults; check `config.power.is_power_saving`.

**Channel security** — `channels.isDefaultChannel(index)` returns true for the public default channel, which gets stricter rate limits. Private channels may have relaxed limits.

## CI/CD

- `main_matrix.yml` — main build pipeline (all PRs and pushes to `master`/`develop`)
- `trunk_check.yml` — linting/formatting gate on PRs
- `test_native.yml` — runs `pio test -e native`
- `tests.yml` — daily end-to-end and hardware-in-the-loop tests
- Matrix targets generated by `bin/generate_ci_matrix.py`

## Resources

- Meshtastic docs: https://meshtastic.org/docs/
- Copilot instructions (more detail on MQTT, CI, common tasks): `.github/copilot-instructions.md`
