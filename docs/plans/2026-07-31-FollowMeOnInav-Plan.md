# FormationFlight — Autonomous Follow Mode — Implementation Plan

**Spec:** [`docs/spec/2026-07-31-FollowMeOnInav.md`](../spec/2026-07-31-FollowMeOnInav.md)
**Status:** Draft for review

This plan sequences the spec into phases that each produce something bench-
(and eventually flight-) testable, moving from hardcoded/compile-time
behavior to fully runtime-editable web UI config, per the incremental
approach requested. Within each phase, nodes with no edge between them can be
built in parallel (by different people or in parallel work sessions); an
edge `A → B` means B genuinely needs A's code to exist, not just "it would be
nice to sequence it that way."

## Confirmed defaults for Phase 1 (hardcoded, compile-time only)

These came out of the pre-planning discussion and are conservative on
purpose — the goal is a safe first bench/flight test, not an operationally
realistic formation distance:

| Key | Value | Source |
|---|---|---|
| `FOLLOW_SLOT_LONG` / `LAT` / `VERT` | `BEHIND` / `CENTER` / `ABOVE` (chase-high) | user (updated: was `LEVEL`/trail — `ABOVE` chosen so the default is testable with the leader stationary on the ground, not just airborne; see spec §7.3) |
| `FOLLOW_GAP_LONG_M` | 15 m | user |
| `FOLLOW_GAP_LAT_M` | 15 m (unused while slot is chase-high, but must have a sane value since the general resolver reads it) | inferred from "wide margins" |
| `FOLLOW_GAP_VERT_M` | 10 m | user |
| `FOLLOW_MIN_SEP_M` | 8 m | user |
| `FOLLOW_MIN_VSEP_M` | 13 m (5 m physical/collision buffer + 8 m worst-case GPS vertical error budget) | user's original 5 m, revised — see "Unit and altitude-frame correctness" below |
| `FOLLOW_MAX_TARGET_DIST_M` | 50 m | user |
| `FOLLOW_TRIGGER_MODE` | `GCSNAV` | user |
| `FOLLOW_TARGET_PEER` | `FIRST_ACTIVE` | user |
| `FOLLOW_EMIT_HZ` | 4 | spec §8 default |
| `FOLLOW_PEER_TIMEOUT_MS` | 1500 | spec §8 default |
| `FOLLOW_MIN_COURSE_SPEED` | 2 m/s (human-facing; compared internally against `peer->gps.groundSpeed`, which is cm/s — see below) | spec §7.5 default |
| `FOLLOW_STATIONARY_MODE` | `HOLD_COURSE` | spec §7.5 default |

All of §9's keys ship as `#define`s (build_flags) in Phase 1. None are
runtime-mutable or EEPROM-persisted until Phase 3.

### Unit and altitude-frame correctness (found in review, fixed here before Phase 1 code lands)

Neither of these has been coded yet — `FollowManager` doesn't exist in the tree
yet (verified: `grep`-ing the codebase for `FollowManager`/`FOLLOW_MIN_VSEP`/
`FOLLOW_MIN_COURSE_SPEED` turns up nothing outside `docs/`). Only
`MSPManager::sendFollowWaypoint()` (0D) exists today, and it just forwards
whatever `alt_cm` it's given — it doesn't compute it, so there's nothing to
fix there. Both issues below are corrected in the spec (§6.2, §7.4, §7.5, §9)
and must be implemented that way from the start in Phase 1 item 6.

1. **Speed unit mismatch.** `peer->gps.groundSpeed` is `int16_t` **cm/s**
   (spec §5[A]), but `FOLLOW_MIN_COURSE_SPEED`'s stationary-leader threshold
   (spec §7.5) is expressed in m/s. Comparing them directly (`groundSpeed <
   FOLLOW_MIN_COURSE_SPEED`) would compare e.g. `200 < 2`, which never trips
   except when the leader is nearly perfectly still — silently defeating the
   §7.5 stationary/low-speed fallback. Fix: keep the config value in m/s
   (human-facing, matches what the web UI will show) but convert at the
   comparison site: `p->gps.groundSpeed < (int16_t)(FOLLOW_MIN_COURSE_SPEED *
   100)`.

2. **Altitude-frame mismatch widens the real vertical error budget.**
   `alt_cm = local_altitude_cm() + peer->relalt*100 + vert_offset*100`
   (spec §6.2) adds the follower's **baro/GPS-fused home-relative** estimate
   (`MSP_ALTITUDE`) to a **raw-GPS-only** delta (`peer->relalt`, computed in
   `PeerManager.cpp:181` from two `MSP_RAW_GPS` altitudes). This is the best
   available signal given the leader broadcasts only raw GPS telemetry (spec
   §1.3 rules out leader-side changes), but it means the effective vertical
   accuracy is bounded by GPS vertical error (commonly ~8 m per receiver,
   worse than horizontal), not by the FC's more accurate baro-fused estimate.
   Fix: `FOLLOW_MIN_VSEP_M` default raised from 5 m to **13 m** (5 m intended
   physical/collision clearance + 8 m worst-case GPS vertical error budget,
   see table above) so the configured "safety margin" isn't silently consumed
   by altitude-source noise. This is a mitigation, not a fix to the
   measurement itself — see the new Phase 1 test item below and spec §13 for
   the longer-term option (leader broadcasting its own home-relative
   altitude).

---

## DAG overview

```
Phase 0 (foundation, all parallel — no edges between them)
  0A PeerManager: id lookup + peer_is_stale()
  0B MSPManager: MSP_ALTITUDE poll + local_altitude_cm()
  0C MSPManager: GCS-NAV-active accessor (getActiveModes() bit 23)
  0D MSPManager: sendFollowWaypoint() emitter
  0E WiFiManager: extend /peermanager/spoof to accept lat/lon/course params (bench-test tooling)
       │
       ▼
Phase 1 (needs 0A, 0B, 0C, 0D; 0E strongly recommended before 1's bench test)
  1  FollowManager: geometry (§7), PeerLock state machine (§6.3), gate (§6.2/§5C),
     sanity guards (§7.4/§7.5), main.cpp registration — compile-time config only
       │
       ├─────────────────────────────┐
       ▼                              ▼
Phase 2 (needs 1)                Phase 2b (needs 1, independent of 2)
  Read-only status endpoint        AUX-channel trigger mode (optional/stretch —
  GET /followmanager/status        spec's "simple first cut" alt. to GCS-NAV;
  (PeerLock state, locked peer,    not required for MVP since Phase 1 already
  last target) — no config write   ships GCS-NAV trigger per chosen defaults)
       │
       ▼
Phase 3 (needs 2; ALSO gated on a
  team decision, see "Blocking
  decision" below)
  3A ConfigHandler: fix `true ||` bug (prerequisite, isolated change)
       │
       ▼
  3B FollowManager: runtime config struct + EEPROM persistence
       │
       ▼
  3C WiFiManager: GET/POST /followmanager/config (validates §7.4 server-side,
     wires FOLLOW_TARGET_PEER change → forced re-acquire per §6.3)
       │
       ▼
Phase 4 (needs 3C; UI layout/component work can start in parallel against a
  mocked JSON contract, but final wiring needs 3C's real endpoint)
  4  html/main.js: new Follow config panel — friendly-grid dropdowns,
     advanced raw-meters toggle, trigger-mode selector, live target-peer
     selector (reads existing /peermanager/status), client-side §7.4 validation
       │
       ▼
Phase 5 (needs 4)
  5  Full acceptance pass: spec §12.1 bench checklist end-to-end with web UI
     editing + reboot-persistence check, then §12.2 progressive flight test
```

### Blocking decision before Phase 3

Spec §13 flags that `ConfigHandler.cpp:37`'s `if (true || cfg.version != VERSION_CONFIG || forcedefault)` may be an intentional "always reset config on boot" behavior that something else currently relies on (e.g. a support workaround), not just a bug. **Confirm with the team before Phase 3A removes it** — this gates all of Phase 3/4 (persistence and the web UI), but does not block Phases 0–2, which are compile-time/read-only and ship independently of this decision.

---

## Phase 0 — Foundation (parallel, no dependencies) - [Completed]

Each of these is a small, independent change to existing files. None require the others to exist first, and each is unit-testable / bench-verifiable in isolation before FollowManager consumes them.

- **0A — `src/lib/Peers/PeerManager.{h,cpp}`**: add a lookup-by-id accessor (peer table is index-based today) and `peer_is_stale(peer_t*)` using `peer->updated` vs. `FOLLOW_PEER_TIMEOUT_MS` (spec §8, §11 item 2).
  *Test:* unit-exercise with the existing 5-peer spoof ring; confirm lookup-by-id returns the right record and staleness flips at the expected time.

- **0B — `src/lib/MSP/MSPManager.{h,cpp}`**: add `MSP_ALTITUDE` (109) polling and cache `estimatedActualPosition` as `local_altitude_cm()` (spec §5[B] — confirmed this doesn't exist anywhere today).
  *Test:* log `local_altitude_cm()` on the bench with the FC at a known height; confirm it's home-relative cm, not the GPS/MSL altitude `MSP_RAW_GPS` would give.

- **0C — `src/lib/MSP/MSPManager.{h,cpp}`**: extend the existing `getActiveModes()`-based state read (already used for the ARM bit in `getState()`) with a "GCS NAV active" accessor via `bitRead(activeModes, MSP_MODE_GCSNAV)` (spec §5[C] option 2).
  *Test:* toggle `GCS NAV` on the bench FC, confirm the accessor flips.

- **0D — `src/lib/MSP/MSPManager.{h,cpp}`**: add `sendFollowWaypoint(lat_1e7, lon_1e7, alt_cm)` using the existing `msp_set_wp_t` struct + `MSP::command()` (spec §6.1 — code sample already given, no hand-rolled framing).
  *Test:* call it manually with a known nearby lat/lon, read back with `MSP_WP` (#254) or watch INAV Configurator's WP list; confirm WP#255 lands correctly and `action=1`.

- **0E — `src/lib/WiFi/WiFiManager.cpp`**: extend `/peermanager/spoof` (or add a parametrized sibling) to accept lat/lon/course per request, mirroring `/gnssmanager/spoof`'s existing param handling, instead of only the fixed 5-peer ring (spec §12.1 note). This isn't strictly required to write Phase 1 code, but doing it now means Phase 1's bench tests (verifying each geometry preset lands in the right place) are repeatable rather than constrained to the fixed ring's known positions.
  *Test:* spoof a peer at a controlled lat/lon/course, confirm it shows up correctly in `/peermanager/status`.

---

## Phase 1 — Core follow module (compile-time config only) [Completed]

**Depends on:** 0A, 0B, 0C, 0D (0E recommended for efficient testing, not a hard code dependency).

This is the first flyable slice: hardcoded chase-high slot (behind, centered, above — testable with the leader on the ground), GCS-NAV gate, lock-on-first-peer with hold-not-failover, full geometry math. Nothing is web-editable yet — config values are `#define`s in a new header (e.g. `src/lib/Follow/FollowConfig.h`), seeded with the table above.

Build the general geometry resolver now rather than a special-cased single-preset version, since the grid→canonical expansion (§7.3) is the same amount of code either way and building it twice would be wasted work — the only thing "hardcoded" about Phase 1 is that the `#define` values can't be changed without a reflash.

Work items (single module, best done by one person/session in sequence since they're all one new file, but listed in the order they naturally get written):

1. `src/lib/Follow/FollowManager.{h,cpp}` — new singleton, registered in `main.cpp`'s `loop()` after `PeerManager`/`GNSSManager`/`MSPManager` (spec §3), self-gated at `1000/FOLLOW_EMIT_HZ` ms, skipped until `sys.phase > MODE_OTA_SYNC`.
2. `follow_switch_active()` — gate on the 0C GCS-NAV accessor.
3. `PeerLock` state machine (spec §6.3): `IDLE` / `ACQUIRING` / `LOCKED` / `LOCKED_HOLDING`, lock-by-id via 0A's lookup, name cross-check mitigation for the id-reuse edge case, no auto-failover on loss.
4. Geometry: `slotToLatLon()` (§7.2), `follow_resolve_offset()` expanding the grid config to canonical meters (§7.3).
5. Altitude: `local_altitude_cm()` (0B) + `peer->relalt` + configured vertical offset (§6.2) — comment the frame assumption at the call site (home-relative + raw-GPS delta, see note above and spec §6.2) so it isn't mistaken for an exact conversion.
6. Safety guards: `follow_target_sane()` — min separation, min vertical separation using the revised `FOLLOW_MIN_VSEP_M = 13 m` default, max target distance (§7.4); leader-stationary fallback using `HOLD_COURSE` (§7.5), with `FOLLOW_MIN_COURSE_SPEED` converted to cm/s before comparing against `peer->gps.groundSpeed` (see note above).
7. Emit via 0D's `sendFollowWaypoint()`.

**Test (bench, props off):** spec §12.1 items 1–7, using 0E's parametrized spoof if available:
- Emitter only fires when GCS NAV is active.
- Multi-peer spoof → locks first peer, ignores others, holds (stops emitting) if the locked peer's updates stop, resumes on same id without re-acquire, re-acquires on switch cycle.
- Computed WP#255 for the default chase-high preset lands at the correct position/altitude relative to the spoofed peer and its course; verify the `×10` scaling fix (§5[A]) empirically — this is called out in the spec as the single most safety-critical fact.
- Freshness guard trips within `FOLLOW_PEER_TIMEOUT_MS`.
- Sub-minimum-separation config refuses to arm.
- Stationary-leader unit check: spoof the leader at ~1.5 m/s and ~3 m/s bracketing the 2 m/s `FOLLOW_MIN_COURSE_SPEED` default and confirm the `HOLD_COURSE` fallback triggers/doesn't at the correct threshold — this is the cm/s-vs-m/s comparison from the note above, easy to get silently wrong.
- Altitude accuracy check: with two real GPS units (not spoofed) at a known, measured height difference, confirm the commanded `alt_cm` lands within `FOLLOW_MIN_VSEP_M` of the intended offset — this characterizes the real-world GPS-vertical-error budget the 13 m default is meant to absorb, rather than only bench-testing `local_altitude_cm()` in isolation (0B's test).

**Test (flight, progressive, open area):** spec §12.2 — chase-high slot (behind/center/above), low speed, manual-override tested first, confirm capture/hold and instant manual recovery. Since the leader can be on the ground for this test, first confirm on the bench that the resolved altitude offset is comfortably clear of the leader's rotor wash/airframe before ever arming.

Phase 1 is a legitimate stopping point if flight validation needs to happen before more work lands — everything past this point is additive.

---

## Phase 2 — Read-only status visibility [Complete]

**Depends on:** Phase 1.

- `GET /followmanager/status` in `WiFiManager.cpp` (spec §10.2, the status half only — no config write): current `PeerLock` state, locked peer id/name, gate active/inactive, last computed target. Backed by a `statusJson()` on `FollowManager`.

This needs no EEPROM and no `ConfigHandler` fix, so it's unblocked regardless of the Phase 3 team decision. It mainly exists to make Phase 1's bench tests less dependent on watching the OSD/Configurator, and to de-risk Phase 4's UI panel (which reuses the same status shape for its live view).

*Test:* poll the endpoint during a bench run, cross-check against OSD/Configurator state.

## Phase 2b — AUX-channel trigger (optional / stretch) [Deferred]

**Depends on:** Phase 1 (needs `follow_switch_active()` to exist as a swappable gate).

Not required for the MVP given the chosen Phase 1 default (GCS-NAV trigger), but the spec documents it as the "simple first cut" alternative (§5[C] option 1) and some pilots may prefer decoupling the follow switch from the FC's nav mode during bench testing. Adds `MSP_RC` polling (genuinely new — unused today) + an AUX accessor, and a `FOLLOW_TRIGGER_MODE=AUX` branch in `follow_switch_active()`. Independent of Phases 2/3 — can be picked up whenever, in parallel with either.

---

## Phase 3 — Runtime config + persistence

**Depends on:** Phase 2 (reuses its status endpoint pattern) **and** the blocking team decision on the `ConfigHandler` reset behavior (see above). Internally sequential (3A → 3B → 3C) since each step needs the previous to compile/function correctly.

- **3A — `src/lib/ConfigHandler.cpp`**: remove the `true ||` short-circuit at line 37, restoring the version-check/force-default logic it was presumably meant to have. Isolated, single-line-scope change; regression-test that normal (non-follow) config still initializes/persists correctly, since this affects the existing `cfg` struct too, not just the new follow config.
- **3B — `src/lib/Follow/FollowManager.{h,cpp}`**: new config struct (all §9 keys), separate from `cfg` (don't overload the existing small/unrelated struct), persisted via the same EEPROM primitives `config_save()`/`config_init()` use, in its own EEPROM region after `cfg`'s footprint. Compile-time `#define`s from Phase 1 become the first-boot seed only.
- **3C — `src/lib/WiFi/WiFiManager.cpp`**: `GET /followmanager/config` (resolved values, grid + canonical view) and `POST /followmanager/config` (accepts subset of §9 keys, validates including server-side §7.4 min-separation check, persists, returns resolved config). Changing `FOLLOW_TARGET_PEER` while the gate is active must force `PeerLock` back to `ACQUIRING` (§6.3's explicit escape hatch).

*Test:* spec §12.1 item 8 — edit a value via `POST`, reboot the follower (or bench unit), confirm the value survived (didn't revert to compile-time default); confirm changing `FOLLOW_TARGET_PEER` mid-flight-mode forces re-acquire.

---

## Phase 4 — Web UI panel

**Depends on:** Phase 3C for real integration. The static layout/component structure can be scaffolded earlier against a mocked JSON response matching 3C's documented contract, but wiring and final testing wait on 3C actually existing.

- New panel in `html/main.js` (or a new component file), modeled on the existing `Settings()` component's structure but wired to `/followmanager/config` and `/followmanager/status` (not the dead `/system/status` path `Settings()` currently uses).
- Friendly-grid dropdowns (§7.3) as the primary editing surface, "advanced" toggle for raw `FOLLOW_OFS_*_M`.
- Trigger-mode selector (only meaningful if Phase 2b landed; otherwise this is a read-only "GCS NAV" label).
- Target-peer selector populated live from the existing `/peermanager/status` endpoint (already exists — no new backend work) — this is also the §6.3 "user changes a setting" escape hatch for peer re-acquire.
- Client-side §7.4 minimum-separation validation that blocks Save (mirrors the server-side check in 3C — both must exist per spec, client-side isn't a substitute for server-side).
- Debounce saves rather than writing on every slider drag, to avoid EEPROM wear (spec §13 open question — resolved here as "yes, debounce").

*Test:* edit each preset from the UI, confirm it takes effect (watch §2's status endpoint or OSD) and persists across reboot.

---

## Phase 5 — Full acceptance pass

**Depends on:** Phase 4 (exercises the complete system, though most of this is re-running earlier bench/flight tests with the UI as the editing surface instead of hardcoded values/raw `POST` calls).

- Full spec §12.1 bench checklist, §12.2 progressive flight checklist, §12.3 acceptance criteria — this time with all geometry presets reachable via the UI rather than reflashes, and persistence verified through the UI's own save path.
- Revisit spec §13's remaining open question: whether the `peer->id` reuse edge case (dropped-and-reassigned LoRa slot during `LOCKED_HOLDING`) is a real risk in practice, based on what multi-peer bench/flight testing showed in Phases 1 and 5.

---

## Files touched, by phase

| File | Phase(s) |
|---|---|
| `src/lib/Peers/PeerManager.{h,cpp}` | 0A |
| `src/lib/MSP/MSPManager.{h,cpp}` | 0B, 0C, 0D, 2b |
| `src/lib/WiFi/WiFiManager.cpp` | 0E, 2, 3C |
| `src/lib/Follow/FollowManager.{h,cpp}` (new) | 1, 2, 3B |
| `src/main.cpp` | 1 |
| `src/lib/ConfigHandler.cpp` | 3A |
| `html/main.js` | 4 |
| `targets/*.ini` | 1 (seed `build_flags`) |

## Open items carried from the spec (not yet resolved by this plan)

- Confirm the `ConfigHandler` `true ||` reset is unintentional before Phase 3A (blocking decision, above).
- `peer->id` reuse edge case — deferred to real multi-peer testing in Phases 1/5, per spec §13.
