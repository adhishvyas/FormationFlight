# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

FormationFlight is ESP32/ESP8266 firmware providing inter-UAS positioning & telemetry
(formation flight, chase footage, ground station coordination) over ESP-NOW or LoRa
(SX127x/SX128x) radios. Spiritual successor to iNav Radar, built on hardware originally
developed for ExpressLRS. Built with PlatformIO + Arduino framework.

## Commands

PlatformIO is installed in `.venv/`; activate it or call the binary directly as `.venv/bin/pio`.

- Build a target: `pio run -e <target>` (e.g. `pio run -e expresslrs_rx_2400_AntennaDiversity_via_WiFi`)
- **Use `expresslrs_rx_2400_AntennaDiversity_via_WiFi` as the default target for test builds** — it's the hardware currently being worked on in this repo.
- List all available targets: `pio project config` or inspect `targets/*.ini` for `[env:...]` sections
- Clean: `pio run -e <target> -t clean`
- Upload/flash: `pio run -e <target> -t upload`
- Serial monitor: `pio device monitor`

There is no unit test suite in this repo — validation is via building targets and, for firmware
changes, on-device or the web UI mock server below.

### Web UI (device dashboard, `html/`)

`html/` is a no-build-step Preact+htm app served by the firmware's AsyncWebServer at `192.168.4.1`.
To preview/test UI changes locally without hardware, use the `web-ui-preview` skill, which runs
`scripts/mock_server.py` — a mock backend that serves `html/` as-is and fakes
the REST endpoints. See that skill's notes: the mock's `validate_config()` is a hand-maintained
mirror of `FollowManager::applyConfig()` and its `DEFAULT_CONFIG` mirrors `configJson()` — keep
both in sync when touching Follow config fields.

## Architecture

### Build system: targets are compile-time configs, not runtime flags

`platformio.ini` only defines shared `[env]`/`env_common_*` bases (per-platform: `esp32`,
`esp32s2`, `esp82xx`; per-radio-band: `433`/`868`/`915`/`915` LoRa, `2400` for SX128x). The actual
buildable environments live in `targets/*.ini` (`diy_espnow.ini`, `diy_lora.ini`, `legacy.ini`,
`expresslrs.ini`), one `[env:...]` per physical board/radio/transport combination. Hardware
capabilities (which radio family, WiFi vs UART config, GNSS support, etc.) are selected via
`build_flags` (`-D HAS_LORA`, `-D LORA_FAMILY_SX128X`, `-D WIFI_CONFIG`, `-D PLATFORM_ESP32`, ...)
baked in per target, not runtime-detected. CI (`.github/workflows/build.yml`) builds every
`UART`/`STLINK` target found across `targets/*.ini` (excluding `DEPRECATED` ones).

Two pre-build scripts run before every compile (wired in `[env] extra_scripts`):
- `scripts/build_html.py` — packs `html/` into a C header + web server handlers, so the web UI
  ships embedded in firmware, not as separate files on the device.
- `scripts/build_flags.py` — injects dynamic build flags (git version/hash, etc.).

### Building a minimal build for OTA updates from a different branch
`PLATFORMIO_BUILD_FLAGS="-D MINIMAL_BUILD" pio run -e expresslrs_rx_2400_AntennaDiversity_via_WiFi`

### Runtime: singleton managers driven from one main loop

`src/main.cpp` owns global state (`cfg`/`config_t`, `sys`/`system_t`, `curr`/`curr_t`, from
`main.h`) and a `setup()`/`loop()` Arduino lifecycle. Nearly all functionality lives in
`src/lib/<Area>/<Area>Manager.{h,cpp}`, each exposing a `getSingleton()` and, for the ones ticked
every cycle, a `loop()`. `main.cpp::loop()` calls managers in a fixed order each cycle, timing each
via `StatsManager`: RadioManager → WiFiManager → PeerManager → GNSSManager → MSPManager →
FollowManager. Below that sits the LoRa OTA sync/TX/RX state machine (`sys.phase`,
`MODE_HOST_SCAN` → `MODE_OTA_SCAN` → `MODE_OTA_SYNC` → `MODE_OTA_RX`/`MODE_OTA_TX`) that handles
peer discovery, slot-timed transmission, and drift correction — this is the timing-critical core
and most fragile part of the loop.

Manager responsibilities:
- `Peers/PeerManager` — tracks other formation members (peer table, liveness).
- `Radios/RadioManager` — abstracts over `ESPNOW`/`LoRa_SX127X`/`LoRa_SX128X`; radio is chosen by
  build-time flags, `RadioManager` dispatches to whichever is enabled.
- `GNSS/GNSSManager` — pluggable location providers/listeners (`MSP_GNSS` reads GPS via the FC over
  MSP; `Direct_GNSS` reads a locally-attached GPS; providers/listeners are registered in
  `main.cpp::setup()` based on `GNSS_ENABLED`/`GNSS_INJECT` flags).
- `MSP/MSPManager` — MSP protocol link to the flight controller (craft name, FC variant detection,
  GNSS injection).
- `Follow/FollowManager` — follow-mode logic (steering the FC toward a target peer); config lives in
  `FollowRuntimeConfig`/`FollowEepromRecord` (`FollowConfig.h`), persisted via EEPROM, editable
  live through the `/followmanager/config` REST endpoint and the web UI's Follow panel. This is the
  area of most active development — see `docs/spec/*FollowMe*.md` and related spec docs for design
  history.
- `Config/ConfigManager` and `lib/ConfigHandler` — EEPROM-backed persisted configuration (distinct
  from the per-manager runtime config structs).
- `WiFi/WiFiManager` — hosts the `AsyncWebServer` REST API (`/system/*`, `/radiomanager/*`,
  `/peermanager/*`, `/mspmanager/*`, `/gnssmanager/*`, `/followmanager/*`, etc.) and serves `html/`;
  this is the API surface the web UI and `mock_server.py` both target.
- `Cryptography/CryptoManager` — encrypts/authenticates OTA radio packets.
- `Display/Display`, `Power/PowerManager`, `Statistics/StatsManager` — OLED UI, power/peripheral
  init, and per-loop-stage timing stats (exposed over `/statsmanager/status`).

### Design docs

`docs/spec/` contains dated design/implementation-plan documents for in-progress features (e.g.
Follow-on-iNav, OSD status GVars, RC-axis control). Check there for the rationale behind ongoing
work before making structural changes in `Follow/` or the OSD/status-reporting paths.
