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

`build_src_filter` in `platformio.ini`'s `[env]` compiles only `main.cpp` and `hal/` (plus
`lib/ff_core` as a PlatformIO library dependency) — see "Runtime" below.

### Tests

Unlike v1, there is a real test suite, all off-hardware. `scripts/run_tests.sh` runs everything,
mirroring `.github/workflows/test.yml`:

```bash
pio test -e native              # lib/ff_core host-native Unity suites (test/test_*)
node --test test/follow-logic.test.js   # html/follow-logic.js
python3 test/test_mock_server.py        # scripts/mock_server.py API + config validation
```

`[env:native]` in `platformio.ini` builds `lib/ff_core` for the host (no Arduino/hardware deps),
so protocol, peer table, rate control, crypto, geodesy, MSP framing, Follow logic, etc. are all
unit-tested independent of any board.

### Web UI (device dashboard, `html/`)

`html/` is a no-build-step Preact+htm app served by `src/hal/WebServer`'s AsyncWebServer at
`192.168.4.1`. The full REST contract is documented in `docs/v2-web-api.md` — read that before
guessing at an endpoint shape. To preview/test UI changes locally without hardware, use the
`web-ui-preview` skill, which runs `scripts/mock_server.py` — a mock backend that serves `html/`
as-is and fakes the REST endpoints against that same contract. `test/test_mock_server.py` checks
the mock's validation rules agree with the native config tests.

## Architecture

**Read `REARCHITECTURE.md` (repo root) before making structural changes.** It's the authoritative
design doc for the v2 rewrite this codebase just went through — protocol rationale, phase-by-phase
status, and a "Deletion inventory" of exactly what v1 machinery is gone and why.

### Runtime: `ff::Node`, not a fixed manager loop

The v1 model this section used to describe — `sys`/`curr`/`cfg` globals, singleton
`<Area>Manager`s called in a fixed order, and a LoRa OTA sync/TX/RX phase machine (`sys.phase`,
`MODE_*`) — is gone. `src/main.cpp` is now thin: `setup()` loads config, builds radio drivers,
groups them in a `RadioHub`, constructs one `ff::Node`, and wires a `FollowController` to it;
`loop()` just services the hub, drains the FC/GPS links, and calls `g_node->poll(now)` and
`g_follow->service(now)`. No blocking calls, no phase state.

The actual application logic lives in `ff::Node` (`lib/ff_core/node.cpp/h`) — pure C++, no
Arduino/hardware includes, fully host-tested. A Node listens continuously (no scan/sync phases —
live at boot), beacons on an ALOHA schedule that adapts per-radio to peer count
(`rate_control.cpp/h`), and folds received frames into a UID-keyed `PeerTable`
(`peer_table.cpp/h`). Everything hardware-shaped is behind an interface the Node depends on:
`IRadioSet` (radio transmit/airtime, implemented by `RadioHub`, `radio_hub.cpp/h`),
`ILocationSource` (own position fix), `ICrypto` (frame encrypt/decrypt), `IMspRadarSink` (push
known peers out over MSP to an FC's OSD or a GCS) — so the whole state machine runs against fakes
on the host.

### `lib/ff_core/` — the pure, host-tested core library

Built by the `[env:native]` PlatformIO environment and linked into every firmware target. Key
modules: `node.cpp/h` (the Node core + its interfaces, above), `protocol.cpp/h` (v2 wire format —
UID-keyed position beacon + announce, clean break from v1, no slot IDs), `radio_hub.cpp/h`
(multi-radio `IRadioSet`, lets ESP-NOW and LoRa transmit/receive simultaneously off one peer
table), `rate_control.cpp/h` + `airtime.cpp/h` (adaptive ALOHA beacon pacing sized from measured
per-radio airtime), `crypto.cpp/h` + `ccm.cpp/h` + `aes.cpp/h` + `sha256.cpp/h` (AES-128-CCM AEAD,
replacing v1's unauthenticated XTS-AES), `config.cpp/h` (the versioned `Settings` document — one
JSON config replacing every v1 EEPROM struct), `scheduler.cpp/h` + `ring_buffer.h` (cooperative
timer scheduler and lock-free SPSC ring the event-driven core is built from), `follow.cpp/h` (see
Follow below), `geo.cpp/h` (geodesy, extracted from v1's `GNSSManager`), `msp_fc.cpp/h` +
`msp_parser.cpp/h` + `msp_crc.h` (non-blocking MSP protocol to the flight controller), `msp_radar.cpp/h`
(builds the `MSP2_COMMON_SET_RADAR_POS` frame for the radar sink), `ubx.cpp/h` (u-blox UBX NAV-PVT
parser for a directly-attached GPS), `sim_traffic.cpp/h` (bench traffic simulator — manufactures
peers for the web UI's Simulator page), `log.cpp/h` + `frame_log.h` (in-RAM log ring and frame log;
the only safe log on ESP8266, where the console UART doubles as the MSP link), `loop_stats.cpp/h`
(per-iteration timing with web-handler time subtracted out, exposed via `docs/v2-web-api.md`'s
`loop` status block).

### `src/hal/` — the hardware adapter layer

Thin adapters wiring `ff_core`'s interfaces to real peripherals: `RadioEspNow`/`RadioSX127x`/
`RadioSX128x`/`SimRadio` (`IRadioSet` drivers added to a `RadioHub`), `MspFcLink` (`ILocationSource`
+ FC link — replaces v1's `MSPManager` and the MSP half of `GNSSManager`), `MspRadarOutput`
(`IMspRadarSink`, runs on its own timer independent of transmit activity or `listen_only`),
`DirectGpsLocationSource` (optional directly-wired GPS, `GNSS_ENABLED` builds), `ConfigStore`
(LittleFS-backed `Settings` persistence — replaces the old EEPROM `ConfigManager`/`ConfigHandler`),
`BoardPower` (PMIC/power-rail bring-up, e.g. the T-Beam's AXP192, ported from v1's `PowerManager`),
`PassthroughCrypto` (the bench "crypto off" escape hatch), `WebServer` (AsyncWebServer REST API +
serves `html/`; contract in `docs/v2-web-api.md` — replaces `WiFiManager`'s REST surface).

### `src/lib/` — legacy v1, unbuilt reference only

`build_src_filter` excludes `src/lib/` entirely from every target; none of it compiles. It's kept
only as reference while each area is ported to the `ff_core`/`hal` model — see REARCHITECTURE.md's
"Remaining / deferred" section for what's still unported (the OLED display stack, as of this
writing). **`src/lib/Follow` specifically is already fully ported** (to `lib/ff_core/follow.cpp/h`
+ `src/hal/MspFcLink`) and is earmarked for deletion — its REST handlers are superseded by the v2
web API. Do not add new work under `src/lib/Follow`; it no longer builds into any target.

### Follow-on-iNav

Follow steering logic now lives in `lib/ff_core/follow.cpp/h` (`FollowController`) — pure,
host-tested, reads the Node's UID-keyed `PeerTable` and an `ILocationSource`, and talks to the FC
through an `IFollowFc` seam implemented by `src/hal/MspFcLink`. Targets are 32-bit UIDs (`0` =
nearest peer with a fix), not v1 slot numbers. Config is part of the unified `ff_core/config.h`
`Settings` document (LittleFS, not the old `FollowEepromRecord` EEPROM path) — edited live via
`POST /api/config` (`docs/v2-web-api.md`) and the web UI's Follow panel (`html/follow.js`). Design
history is in `docs/spec/*FollowMe*.md` and sibling docs under `docs/spec/` (status/OSD GVars,
RC-axis control, speed autothrottle, the test suite) and `docs/plans/*-Plan.md`; REARCHITECTURE.md's
"Phase 1h" entry documents exactly how the v1 PR was ported onto the Node core, which is worth
reading before touching this area structurally.

### Design docs

`docs/spec/` holds dated design/implementation-plan documents for in-progress features;
`docs/plans/` holds matching phased execution plans for some of them. `REARCHITECTURE.md` is the
design doc for the v2 rewrite itself and supersedes any v1-era architectural assumptions elsewhere
in these docs — if a spec doc and REARCHITECTURE.md disagree on how something is wired, the
rearchitecture doc wins.
