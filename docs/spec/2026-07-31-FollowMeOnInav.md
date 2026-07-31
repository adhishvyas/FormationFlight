# FormationFlight — Autonomous Follow Mode (Option B) — Engineering Spec

**Status:** Draft for planning — v2, revised against source (see changelog)
**Target firmware:** FormationFlight (ESP32/ESP8266, PlatformIO/C++)
**Follower FC:** INAV, multirotor (quadcopter) only
**Source of truth for the flight controller side:** INAV `GCS NAV` follow-me via MSP `MSP_SET_WP` (#209), special waypoint #255

**Changelog since v1:**
- Corrected §5 field names/scaling/altitude claims and §6.1 emitter code against the actual `PeerManager`/`MSPManager`/`MSP` source (see inline notes).
- Added §6.3: peer selection is now a lock-on-first-detected state machine, not a per-cycle "pick first active" scan (addresses requirement: don't fail over to a second peer if the locked one is lost).
- Added §10: follow parameters are runtime-configurable via the web UI, not compile-time-only (addresses requirement: web UI configurability). This supersedes v1's "config may be compile-time initially" framing.

---

## 1. Purpose & Scope

### 1.1 Goal
Add a mode to the FormationFlight (FF) firmware such that, when the pilot engages a switch on the follower aircraft, the follower autonomously flies to and holds a configurable position relative to a peer aircraft ("the leader"), using the peer position data FF already receives over its radio link. Position is expressed as a 3D offset (distance + relative location: behind, right, above, etc.).

### 1.2 In scope
- Modifying FF firmware to emit `MSP_SET_WP` (#255) to the follower's FC.
- Reading the leader's position from FF's existing peer table.
- Gating the emitter on a pilot-controlled switch / flight mode.
- A configurable follow-geometry section (distance + relative location).
- Freshness and sanity safety guards inside FF.
- **Single-leader lock-on behavior:** when multiple peers are visible, follow only the first peer detected at the moment follow mode is engaged; do not automatically retarget to a different peer if the locked one is lost (§6.3).
- **Runtime configuration via the web UI:** all follow parameters (geometry, safety bounds, trigger mode, target-peer selection) are readable and writable from FF's existing web UI and persist across reboot (§10).

### 1.3 Out of scope (this iteration)
- Fixed-wing followers. The follower is assumed to be a multirotor that can hold and reposition. Fixed-wing trail flight is explicitly deferred.
- Multi-follower coordination / collision avoidance between multiple followers.
- Leader-side changes. The leader only needs to be broadcasting position over FF as it does today.
- Any change to INAV firmware. We use INAV's existing `GCS NAV` mechanism unmodified.
- **Automatic failover to an alternate peer** when the locked leader is lost. This is a deliberate non-goal, not an oversight — see §6.3 for the rationale (predictability/safety over availability).

### 1.4 Assumptions
- Both aircraft already run INAV + FF and can see each other in the HUD (peer telemetry is working end-to-end).
- The follower's FC has a working GPS 3D fix, calibrated compass, and flies `NAV POSHOLD` cleanly on its own.
- FF on the follower is connected to the FC over an MSP-configured UART (the same link used today for the radar HUD).

---

## 2. Background / Why This Design

- **FF is display-only today.** FF relays peer position/altitude/speed/name to the FC purely for OSD/HUD display. It does not command navigation. There is no stock path from peer data into the follower's navigation controller.
- **INAV already has a follow-me hook: `GCS NAV`.** With the FC in `NAV POSHOLD` + `GCS NAV`, an external consumer repeatedly writes special waypoint **#255** (the POSHOLD target) via `MSP_SET_WP` (#209). INAV flies to that moving point. This is the mechanism we drive.
- **Modify FF firmware** because everything needed already lives in FF: the peer table, the follower's own position, and the MSP-writing plumbing (FF already frames and writes MSP to the FC, including GPS injection via `MSP2_COMMON_SET_RADAR_POS`). We add one outbound message and a small decision task; no extra UART or companion computer.
- **FF's config today is compile-time only**, set via PlatformIO `build_flags` in `targets/*.ini`. There is no working runtime-persisted config and no config-write web endpoint anywhere in FF (verified — see §10.1). Making follow parameters web-configurable (§1.2) is new infrastructure, not a reuse of an existing mechanism, and depends on fixing a pre-existing EEPROM bug (§10.1) as a prerequisite.

---

## 3. High-Level Architecture

```
                 FF radio link (LoRa / ESP-NOW)
   Leader FF  ───────────────────────────────────▶  Follower FF (ESP32)
                                                      │
                                    ┌─────────────────┼────────────────────┐
                                    │ PeerManager      │ MSPManager        │
                                    │  peer table      │  MSP TX/RX to FC  │
                                    └─────────┬────────┴─────────┬─────────┘
                                              │                  │
                                        [A] peer pos       [B] local alt / [C] switch state
                                              │                  │
                                          ┌───▼──────────────────▼────┐
                                          │   follow module (NEW)     │
                                          │  - gate on switch/mode    │
                                          │  - lock peer, freshness   │
                                          │  - compute 3D slot target │
                                          │  - emit MSP_SET_WP #255 │
                                          └───────────┬───────────────┘
                                                      │ MSP_SET_WP (#209, wp=255)
                                                      ▼
                                            Follower INAV FC
                                     (NAV POSHOLD + GCS NAV active)
```

The pilot's switch position engages `NAV POSHOLD` + `GCS NAV` on the FC. FF detects the follow condition, locks onto a leader, computes the target slot behind/around it, and streams it to WP#255. The FC flies there and holds.

**Runtime pattern note (verified against source):** FF has **no task-scheduler abstraction**. `src/main.cpp`'s `loop()` calls each manager singleton's `.loop()` unconditionally every iteration, in a fixed order (`RadioManager` → `WiFiManager` → `PeerManager` → `GNSSManager` → `MSPManager`), and each manager self-throttles internally with a `millis()` check (e.g. `PeerManager::loop()` gates its body on a 100 ms interval; `MSPManager::loop()` gates on a `nextSendTime` tied to the LoRa TDMA slot). The new follow module should be a new singleton (e.g. `FollowManager`) added as one more `.loop()` call at the end of that sequence — after `PeerManager`, `GNSSManager`, and `MSPManager` have all run so a fresh peer, fresh self-location, and open MSP link are available that iteration — self-gated the same way at `1000 / FOLLOW_EMIT_HZ` ms, and skipped until `sys.phase > MODE_OTA_SYNC` (mirroring `MSPManager::loop()`'s own gate).

---

## 4. Follow-Me Flight-Controller Contract (INAV side, fixed)

These values are dictated by INAV and must not be changed:

- **Modes required active on follower:** `NAV POSHOLD` **and** `GCS NAV` together.
- **Message:** `MSP_SET_WP`, command **209**.
- **Waypoint number:** **255** (POSHOLD/follow-me slot).
- **action:** **1** (`MSP_NAV_STATUS_WAYPOINT_ACTION_WAYPOINT`, already `#define`d in `src/lib/MSP/MSP.h:410`). Setting action = 0 makes the message invalid.
- **lat / lon:** `int32`, degrees × **1e7**.
- **alt:** `int32`, **centimeters**. Home-relative when p3 bit0 = 0.
- **p1, p2, p3, flag:** all **0** for our use (p1 doubles as "speed cm/s" per INAV but we don't set it; p3 bit0 = 0 ⇒ altitude relative to home; do not set AMSL).

Payload layout (21 bytes, little-endian) — **this struct already exists in FF's codebase**, verbatim, at `src/lib/MSP/MSP.h:670-680` as `msp_set_wp_t` (packed):

| Offset | Field           | Type   | Value                         |
|-------:|-----------------|--------|--------------------------------|
| 0      | waypointNumber  | uint8  | 255                            |
| 1      | action          | uint8  | 1                              |
| 2      | lat             | int32  | latitude × 1e7                 |
| 6      | lon             | int32  | longitude × 1e7                |
| 10     | alt             | int32  | altitude in cm (home-relative) |
| 14     | p1              | int16  | 0                               |
| 16     | p2              | int16  | 0                               |
| 18     | p3              | int16  | 0                               |
| 20     | flag            | uint8  | 0                               |

MSPv1 frame: `$ M < [len=21] [cmd=209] [payload...] [crc]`. FF already has the framing/CRC/ACK-wait plumbing for this in `MSP::command()` (`src/lib/MSP/MSP.cpp:244`) — do not hand-roll a new frame writer (see §6.1).

---

## 5. Data Sources Inside FF (integration seams) — verified against source

Three seams bind to real symbols in the current FF checkout, confirmed by reading `PeerManager`, `MSPManager`, and `MSP`.

### [A] Leader position — PeerManager
File: `src/lib/Peers/PeerManager.h`/`.cpp`. The peer record is `peer_t` (`PeerManager.h:9-29`). **Correction vs. v1 of this spec:** there are no top-level `lat`/`lon`/`latRaw`/`lonRaw`/`groundCourse`/`relativeAltitude`/`age` fields on `peer_t`. Those names are **web-JSON-only**, produced on the fly by `PeerManager::statusJson()` (`PeerManager.cpp:193-234`). The real internal fields firmware code must use are:

| Internal field                  | JSON name           | Type    | Notes |
|----------------------------------|----------------------|---------|-------|
| `peer->gps.lat`, `peer->gps.lon` | `lat`/`lon`, `latRaw`/`lonRaw` | `int32_t` | see scaling note below |
| `peer->gps.groundCourse`         | `groundCourse`       | `int16_t` | **degrees × 10**, not plain degrees — divide by 10 before use (precedent: `MSP_GNSS.cpp:38`, `loc.groundCourse = rawLocation.groundCourse / 10;`) |
| `peer->gps.groundSpeed`          | `groundSpeed`        | `int16_t` | cm/s |
| `peer->relalt`                   | `relativeAltitude`   | `int16_t` | leader alt minus follower alt, meters (computed in `PeerManager::loop()`, `PeerManager.cpp:162`, from two `GNSSLocation.alt` doubles — **not home-relative**, just a raw GPS/baro altitude difference) |
| `peer->updated`                  | `updated`             | `uint32_t` | millis() timestamp |
| `peer->lost`                     | `lost`                | `uint8_t`  | set to 2 when `millis() - peer->updated > LORA_PEER_TIMEOUT` (`main.h:51`, **6000 ms**) |
| `peer->id`                       | `id`                  | `uint8_t`  | LoRa slot id, assigned once, stable for the session (see §6.3 for lock semantics and its caveat) |
| (none — computed only in JSON)   | `age`                 | —       | `millis() - peer->updated`; no stored equivalent exists yet |

- **⚠ Scaling gotcha (confirmed, and the exact factor differs from what v1 assumed):** `peer->gps.lat`/`.lon` are `int32_t` scaled **×1e6** in practice. The struct's own header comment (`msp_raw_gps_t`, `MSP.h:369-370`) claims ×1e7, but the working code that actually uses this field for outbound MSP — `MSPManager::sendRadar()` (`MSPManager.cpp:219-220`) — does `position.lat = peer->gps.lat * 10; // x 10E7`, i.e. it treats the stored value as ×1e6 and multiplies by 10 to reach INAV's native ×1e7. **The follow emitter must do the same `* 10`**, mirroring `sendRadar()`'s proven behavior, not the (incorrect) struct comment. Getting this wrong scales the target ~10× toward 0°/0° → guaranteed flyaway. This is the single most safety-critical fact in the spec; verify empirically on the bench (§11.1 item 4) before any flight regardless.
- **Peer selection is no longer "scan every cycle for the first active peer."** See §6.3 — it's a lock-on-acquire state machine.

### [B] Follower's own altitude — MSPManager
**Correction vs. v1:** v1 claimed FF "already fetches" a home-relative self-altitude and this could be reused. **Confirmed false — no such stored value exists anywhere in FF today.** `MSPManager::getLocation()` (`MSPManager.cpp:135-149`) does an uncached `MSP_RAW_GPS` request, and `msp_raw_gps_t.alt` is meters, typically GPS/MSL — not home-relative, not cached. The struct that actually holds INAV's home/baro-relative estimate, `msp_altitude_t` (`estimatedActualPosition`, cm, `MSP.h:275-279`) and its command `MSP_ALTITUDE` (`#define MSP_ALTITUDE 109`, `MSP.h:51`), **exist but are never requested anywhere in the current codebase** (zero call sites). This is new work: add a poll of `MSP_ALTITUDE` in `MSPManager` and cache `estimatedActualPosition` as `local_altitude_cm()`.

### [C] Switch / mode state — MSPManager
Two supported triggers, as in v1, but **simplify the "preferred" one against what already exists:**

1. **AUX-channel (simple first cut):** genuinely new — FF does not poll `MSP_RC` anywhere today (only the unused `#define MSP_RC 105` / `msp_rc_t` struct exist). Would need new polling + an AUX accessor.
2. **GCS-NAV-reactive (preferred, safer):** **simpler than v1 assumed.** `MSP::getActiveModes(uint32_t *activeModes)` (`MSP.cpp:304-330`) already implements the classic `MSP_STATUS` + `MSP_BOXIDS` flow and is already called today by `MSPManager::getState()` (`MSPManager.cpp:29-38`) to read the ARM bit for OSD/HUD status. The same bitmap already includes `MSP_MODE_NAVPOSHOLD` (bit 9) and `MSP_MODE_GCSNAV` (bit 23) — both `#define`d in `MSP.h:96,110`. **Drop the `MSP2_INAV_STATUS` idea from v1 entirely**; there's no need for an INAV-specific v2 message. Triggering on "GCS NAV active" is a one-line `bitRead(activeModes, MSP_MODE_GCSNAV)` addition reusing plumbing that's already proven in production for the arm-state case.

Ship #1 to get flying; move to #2 for the production behavior (unchanged recommendation from v1, now cheaper to implement than v1 assumed).

---

## 6. New Firmware Components

### 6.1 MSP emitter (add to MSPManager)
**Correction vs. v1:** the v1 code sample hand-rolled a byte buffer and an inline XOR checksum. Both already exist in FF as reusable, tested infrastructure — `msp_set_wp_t` (§4) and `MSP::command()` (`MSP.cpp:244`, which frames, computes the checksum, and writes for you). Use them instead of duplicating:

```cpp
// MSP_SET_WP (#209) — INAV follow-me special waypoint #255.
// Requires NAV POSHOLD + GCS NAV active on the follower FC.
void MSPManager::sendFollowWaypoint(int32_t lat_1e7, int32_t lon_1e7, int32_t alt_cm) {
    msp_set_wp_t wp{};
    wp.waypointNumber = 255;
    wp.action = MSP_NAV_STATUS_WAYPOINT_ACTION_WAYPOINT; // must be 1
    wp.lat = lat_1e7;
    wp.lon = lon_1e7;
    wp.alt = alt_cm;         // home-relative, p3 bit0 = 0 below
    wp.p1 = 0; wp.p2 = 0; wp.p3 = 0;
    wp.flag = 0;
    msp->command(MSP_SET_WP, &wp, sizeof(wp));
}
```

### 6.2 Follow task (new module `follow`)
Registered as a self-gated `.loop()` per §3, targeting **~4 Hz** (see §8 rate caution).

```cpp
void FollowManager::loop() {
    if (!follow_switch_active())   { peerLock.clear(); return; }   // [C] gate — also drops any lock (§6.3)
    peer_t *p = peerLock.resolve();                                // [A] peer lock/acquire (§6.3)
    if (!p || peer_is_stale(p))    return;                         // freshness guard (§7.4/§8) — HOLD, do not re-target

    // Resolve configured slot -> track-relative meters (§7 geometry)
    FollowOffset o = follow_resolve_offset();           // long/lat/vert meters
    LL tgt = slotToLatLon(p->gps.lat * 10, p->gps.lon * 10, p->gps.groundCourse / 10.0,
                          o.longitudinal_m, o.lateral_m);

    // NOTE on frame mixing: local_altitude_cm() is the follower's baro/GPS-fused
    // home-relative estimate (MSP_ALTITUDE); p->relalt is a raw-GPS-only delta
    // (MSP_RAW_GPS altitude, leader minus follower — PeerManager.cpp). Summing
    // them is the best available approximation given the leader only ever
    // broadcasts raw GPS telemetry (§1.3 rules out leader-side changes), but the
    // result's real accuracy is bounded by GPS vertical error (commonly ~8 m per
    // receiver), not by the FC's more accurate baro-fused estimate. This is why
    // FOLLOW_MIN_VSEP_M (§7.4) carries an explicit GPS-error margin, not just a
    // physical-clearance buffer.
    int32_t alt_cm = local_altitude_cm()                 // [B] home-relative, baro/GPS-fused
                   + (int32_t)(p->relalt * 100)           // leader vs follower, raw-GPS delta, m->cm
                   + (int32_t)(o.vertical_m * 100);       // configured vertical offset

    if (!follow_target_sane(p, tgt, alt_cm)) return;    // sanity bounds (§7.4)
    mspManager->sendFollowWaypoint(tgt.lat_1e7, tgt.lon_1e7, alt_cm);
}
```

### 6.3 Peer Selection & Lock State Machine (NEW — addresses "follow only the first-detected aircraft")

**Requirement:** when FF is tracking multiple peer aircraft and the pilot enables follow mode, the follower must lock onto and follow only the **first** peer detected at that moment. If the locked peer is subsequently lost, the follower must **not** automatically retarget to a different peer — it falls back to the standard stale-peer procedure (§8: stop emitting, FC holds last WP#255) until either the same locked peer's telemetry is regained, or the user explicitly changes the target-peer setting (or cycles the follow switch).

This is a deliberate design choice, not an omission: silently retargeting to a different aircraft mid-flight would change the follower's commanded position without any pilot action, which is exactly the kind of surprise behavior a formation-flying safety design should avoid. Predictability (stay locked, hold if lost) is preferred over availability (always chase *some* peer).

**State machine** (owned by a new `PeerLock` helper inside the follow module):

| State | Entered when | Behavior |
|---|---|---|
| `IDLE` | Follow gate (switch/`GCS NAV`) inactive | No lock held. `follow_update()` no-ops. |
| `ACQUIRING` | Gate just became active, no lock yet | Scan the peer table once; lock onto the **first** peer with `id > 0` and not `lost` (or, if `FOLLOW_TARGET_PEER` names an explicit id, lock onto that id once it's present and fresh). Locking stores `peer->id` (and `peer->name` as a sanity cross-check — see caveat below), not an array index. |
| `LOCKED` | A peer is locked and its telemetry is fresh | Emit WP#255 tracking that specific peer id every cycle, per §6.2. |
| `LOCKED_HOLDING` | The locked peer's telemetry goes stale (`peer_is_stale()`, §8) or `lost` | **Stop emitting.** Do not scan for or switch to another peer. FC holds the last commanded WP#255 (standard §8 procedure). Keep checking the *same* locked id for freshness every cycle. |
| → back to `LOCKED` | The same locked peer id becomes fresh again | Resume emitting to that peer without re-acquiring — this is "regaining" the original leader, not picking a new one. |
| → `IDLE` | Gate goes inactive (switch off, or `GCS NAV` drops) | Clear the lock unconditionally. Re-engaging always re-runs `ACQUIRING` and can lock a different (or the same) first-detected peer. |
| → `ACQUIRING` | User changes `FOLLOW_TARGET_PEER` (via web UI, §10) while gate is active | Clear the lock and re-acquire immediately under the new setting. This is the explicit "user changes a setting" escape hatch. |

**Caveat (flag, don't silently patch around):** `peer->id` is a small LoRa slot id assigned once via `pick_id()` and is stable for as long as that peer keeps its slot, but if a peer drops out long enough that FF reassigns its slot to a *different* physical aircraft, the same `id` could now refer to someone else. Mitigate by also checking `peer->name` still matches what was locked before resuming emission from `LOCKED_HOLDING`; if the name changed under the same id, treat it as a lost lock (fall through to `IDLE`-style safe behavior) rather than silently following a new aircraft under the old id. This should be confirmed against real multi-peer bench testing (§11.1) before flight.

`FOLLOW_TARGET_PEER = FIRST_ACTIVE` (default) uses the "first peer seen at acquire time" rule above. `FOLLOW_TARGET_PEER = <peer id>` (settable via web UI once peers are visible, §10) skips the "first" ambiguity entirely by pinning acquisition to a specific id from the start; the hold-on-loss / no-failover behavior is identical either way.

---

## 7. Follow Geometry (configurable distance + relative location)

This is the configurable positioning section. The follower's slot is a 3D offset from the leader, expressed in the **leader's track-relative frame**, so the formation slot rotates with the leader's heading (the way real formation flight works).

### 7.1 Coordinate frame (authoritative definition)
Origin = leader's current position. Axes:

- **Longitudinal (x):** along the leader's `groundCourse`. **+ = ahead** (direction of travel), **− = behind**.
- **Lateral (y):** perpendicular to course. **+ = right** (starboard, 90° clockwise from course), **− = left** (port).
- **Vertical (z):** **+ = above**, **− = below**.

A slot is the triple `(longitudinal_m, lateral_m, vertical_m)`.

### 7.2 Geometry math (reference implementation)
θ = leader `groundCourse` in **plain degrees** (clockwise from north) — remember to divide FF's internal `peer->gps.groundCourse` by 10 first (§5[A]).

```cpp
struct LL { int32_t lat_1e7, lon_1e7; };

// long_m: +ahead/-behind ; lat_m: +right/-left
LL slotToLatLon(int32_t plat_1e7, int32_t plon_1e7, float course_deg,
                float long_m, float lat_m) {
    double lat = plat_1e7 / 1e7, lon = plon_1e7 / 1e7;
    double th  = radians(course_deg);
    double north_m = long_m * cos(th) - lat_m * sin(th);   // ahead unit + right unit
    double east_m  = long_m * sin(th) + lat_m * cos(th);
    double dlat = north_m / 111320.0;
    double dlon = east_m  / (111320.0 * cos(radians(lat)));
    return { (int32_t)lround((lat + dlat) * 1e7),
             (int32_t)lround((lon + dlon) * 1e7) };
}
```

Sanity checks of the frame (leader heading north, θ=0): behind ⇒ due south; right ⇒ due east; front-right ⇒ northeast. Vertical is applied separately in `alt_cm` (§6.2).

### 7.3 Configuration model

**Canonical config = three signed meter values** (everything else expands to these):

| Key                     | Type  | Meaning                         |
|-------------------------|-------|---------------------------------|
| `FOLLOW_OFS_LONG_M`     | float | + ahead / − behind the leader   |
| `FOLLOW_OFS_LAT_M`      | float | + right / − left of the leader  |
| `FOLLOW_OFS_VERT_M`     | float | + above / − below the leader    |

**Friendly grid (convenience layer)** — pick one enum per axis plus a gap distance per axis; it expands to the canonical meters. This is what expresses "above right", "bottom left", "trail", etc., and is also what the web UI (§10) presents as the primary editing surface.

| Key                    | Values                     | Expands to                               |
|------------------------|----------------------------|-------------------------------------------|
| `FOLLOW_SLOT_LONG`     | `AHEAD` / `CENTER` / `BEHIND` | `+gap_long` / `0` / `−gap_long`       |
| `FOLLOW_SLOT_LAT`      | `LEFT` / `CENTER` / `RIGHT`   | `−gap_lat` / `0` / `+gap_lat`         |
| `FOLLOW_SLOT_VERT`     | `BELOW` / `LEVEL` / `ABOVE`   | `−gap_vert` / `0` / `+gap_vert`       |
| `FOLLOW_GAP_LONG_M`    | float                      | longitudinal spacing when AHEAD/BEHIND   |
| `FOLLOW_GAP_LAT_M`     | float                      | lateral spacing when LEFT/RIGHT          |
| `FOLLOW_GAP_VERT_M`    | float                      | vertical spacing when ABOVE/BELOW        |

**Default slot:** `FOLLOW_SLOT_LONG = BEHIND`, `FOLLOW_SLOT_LAT = CENTER`, `FOLLOW_SLOT_VERT = ABOVE` (i.e. `chase-high`, resolving to `(−H, 0, +V)`). Chosen as the factory default because it keeps the follower clear of the leader's downwash/rotor disc while still being testable with the leader stationary on the ground — a purely horizontal default (e.g. `trail`) would fly the follower into the ground at the leader's altitude when the leader isn't airborne, whereas `BEHIND/CENTER/ABOVE` gives useful separation on the bench and in early flight tests regardless of whether the leader is flying or grounded.

**Named preset examples** (using per-axis gaps; horizontal `H`, vertical `V`):

| Descriptor        | LONG   | LAT    | VERT   | Resulting slot (long, lat, vert)   |
|--------------------|--------|--------|--------|-------------------------------------|
| `trail`           | BEHIND | CENTER | LEVEL  | (−H, 0, 0)                         |
| `lead`            | AHEAD  | CENTER | LEVEL  | (+H, 0, 0)                         |
| `line-abreast-right` | CENTER | RIGHT | LEVEL | (0, +H, 0)                        |
| `line-abreast-left`  | CENTER | LEFT  | LEVEL | (0, −H, 0)                        |
| `echelon-rear-right` | BEHIND | RIGHT | LEVEL | (−H, +H, 0)                       |
| `echelon-rear-left`  | BEHIND | LEFT  | LEVEL | (−H, −H, 0)                       |
| `above-right`     | CENTER | RIGHT  | ABOVE  | (0, +H, +V)                        |
| `bottom-left`     | CENTER | LEFT   | BELOW  | (0, −H, −V)                        |
| `overhead`        | CENTER | CENTER | ABOVE  | (0, 0, +V)                         |
| `chase-high`      | BEHIND | CENTER | ABOVE  | (−H, 0, +V)                        |

Resolution order at runtime: if the friendly grid keys are set, expand them into `FOLLOW_OFS_*_M`; otherwise use `FOLLOW_OFS_*_M` directly. `follow_resolve_offset()` returns the resolved triple.

### 7.4 Geometry safety rules
- **Minimum 3D separation.** Reject/never-arm a slot whose magnitude `sqrt(long² + lat² + vert²)` is below `FOLLOW_MIN_SEP_M`. This forbids the degenerate `CENTER/CENTER/LEVEL` (collision) slot. The web UI (§10) must enforce this client-side too (block Save, not just firmware-side reject) so a bad config never gets persisted in the first place.
- **Minimum vertical gap for stacked slots.** If both horizontal components are ~0 (overhead/underneath), require `|vert| ≥ FOLLOW_MIN_VSEP_M` to avoid a vertical GPS-noise collision. **Default raised to 13 m** (5 m intended physical/collision clearance + 8 m worst-case GPS vertical error budget), because `alt_cm`'s accuracy is bounded by raw GPS vertical error, not by the FC's baro-fused estimate — see §6.2's note on altitude-frame mixing. Do not treat this value as pure physical clearance; it's absorbing measurement noise as well as providing separation.
- **Runtime sanity (`follow_target_sane`).** Reject the computed target if the leader's reported distance is implausibly large, if the solved target is unreasonably far from the follower, or if `groundCourse` is unusable (e.g. leader effectively stationary — see §7.5).

### 7.5 Leader-stationary / low-speed handling
`groundCourse` is meaningless when the leader is nearly stationary, so a track-relative slot cannot be oriented. When leader `groundSpeed` < `FOLLOW_MIN_COURSE_SPEED` (e.g. 2 m/s), fall back to one of:
- **Hold last valid course** (freeze the slot orientation from the last time speed was valid), or
- **Compass/world-frame offset** (treat lateral/longitudinal as fixed N/E) as a configured fallback.
Default: hold last valid course. Configurable via `FOLLOW_STATIONARY_MODE`.

**⚠ Unit gotcha:** `FOLLOW_MIN_COURSE_SPEED` is expressed in m/s (human-facing, matches the web UI), but `peer->gps.groundSpeed` is `int16_t` **cm/s** (§5[A]). Comparing them directly compares e.g. `200 < 2`, which almost never trips — silently defeating this fallback. Convert at the comparison site:
```cpp
if (p->gps.groundSpeed < (int16_t)(FOLLOW_MIN_COURSE_SPEED * 100)) { /* stationary fallback */ }
```

---

## 8. Timing, Rate, and Failsafe

- **Emit rate:** ~4 Hz. Do not flood MSP; INAV has limited buffering and can drop messages. Peer updates arrive only a few times per second regardless.
- **Freshness guard (`peer_is_stale`):** the locked peer is stale if `millis() - peer->updated > FOLLOW_PEER_TIMEOUT_MS` (default 1500 ms) **or** `peer->lost` is set. **This is intentionally tighter than FF's own general peer-loss threshold** (`LORA_PEER_TIMEOUT = 6000 ms`, `main.h:51`, which drives the HUD's "lost" flag) — 6 s of staleness is far too old to keep commanding a moving formation slot, so the follow module runs its own tighter check directly against `peer->updated` rather than waiting for `peer->lost` to flip. Never re-command a stale point.
- **Stop = hold, then FC recovers.** When emission stops (stale peer → `LOCKED_HOLDING`, §6.3), INAV holds the last WP#255. The real recovery is the pilot's switch: flipping it drops `GCS NAV` on the FC regardless of FF, reverting to plain `POSHOLD`/manual. The FC mode is authoritative. This is also the mechanism behind §6.3's "no automatic peer failover" rule — losing the leader degrades to a hold, not a retarget.
- **Manual override:** a distinct, always-available switch/mode must let the pilot take control instantly. This is an FC-side concern (standard INAV mode setup), reaffirmed here as a requirement.
- **Startup:** emitter must not send until (a) follower has 3D fix + valid home, (b) a fresh valid peer exists, and (c) the gate is active.

---

## 9. Configuration Summary (all keys)

These are the parameters that exist both as compile-time defaults (`build_flags`, per §2) **and** as runtime-editable values via the web UI (§10) — the web UI is the primary way these get changed after initial flashing; the compile-time values only matter as factory defaults / first-boot seed.

Geometry (§7.3): `FOLLOW_OFS_LONG_M`, `FOLLOW_OFS_LAT_M`, `FOLLOW_OFS_VERT_M`, or grid `FOLLOW_SLOT_LONG/LAT/VERT` + `FOLLOW_GAP_LONG_M/LAT_M/VERT_M` (default slot: `BEHIND`/`CENTER`/`ABOVE`, i.e. `chase-high` — testable with the leader on the ground).

Behavior / safety:
- `FOLLOW_TRIGGER_MODE` = `AUX` | `GCSNAV` (§5[C])
- `FOLLOW_AUX_CH`, `FOLLOW_AUX_US` (when trigger = AUX)
- `FOLLOW_TARGET_PEER` = `FIRST_ACTIVE` | `<peer id>` — governs §6.3's lock-acquire rule; changing this at runtime forces a fresh acquire (§6.3)
- `FOLLOW_EMIT_HZ` (default 4)
- `FOLLOW_PEER_TIMEOUT_MS` (default 1500)
- `FOLLOW_MIN_SEP_M`, `FOLLOW_MIN_VSEP_M` (default **13 m** — 5 m physical/collision clearance + 8 m worst-case GPS vertical error budget; see §7.4)
- `FOLLOW_MIN_COURSE_SPEED` (m/s; converted to cm/s at comparison time against `peer->gps.groundSpeed` — see §7.5), `FOLLOW_STATIONARY_MODE` = `HOLD_COURSE` | `WORLD_FRAME`
- `FOLLOW_MAX_TARGET_DIST_M` (runtime sanity bound)

---

## 10. Web UI Configuration (Runtime) (NEW — addresses "follow parameters configurable via web UI")

### 10.1 Why this is new work, not a reuse
Verified against source: FF's config today is **compile-time only in practice**.
- `src/main.h` defines a global `config_t cfg` with a handful of unrelated fields (`force_gs`, `lora_nodes`, `slot_spacing`, `lora_timing_delay`, `msp_after_tx_delay`, `display_enable`) — no follow-related fields, and no generic key/value or JSON layer.
- `config_init()` in `src/lib/ConfigHandler.cpp:27-46` reads `cfg` from EEPROM but then **unconditionally overwrites it with hardcoded defaults** every boot, due to `if (true || cfg.version != VERSION_CONFIG || forcedefault)` (`ConfigHandler.cpp:37`) — a literal `true ||` that short-circuits the version check. **Any EEPROM-persisted edit to `cfg` is discarded on every boot today.** This is a pre-existing bug, unrelated to follow mode, but it directly blocks persisting follow config the same way — **fixing it (removing `true ||`) is a prerequisite for this section**, not optional polish.
- There is no HTTP endpoint anywhere in `WiFiManager.cpp` that writes/persists config. All existing `POST` endpoints are either fire-and-forget actions (`/peermanager/spoof`, `/radiomanager/radio_set_enabled`, `/system/reboot`) or spoof/debug setters that live only in RAM (`/gnssmanager/spoof`).
- `html/main.js` has a `Settings()` component (lines ~163-199) that *looks* like a config UI (fields for `log_enabled`, `log_level`, `brightness`, `device_name`) but is vestigial: its save handler issues a `GET /system/status` with the JSON body **commented out**, and none of those fields exist in the actual `/system/status` response. It is not a usable pattern to extend as-is, though its layout/component structure is a reasonable starting point for a new panel.

### 10.2 Design
- **New `FollowManager` config struct**, holding every key in §9, distinct from the existing unrelated `config_t cfg` (don't overload that struct — it's small, unrelated, and already fragile per the bug above). Persist it via the same EEPROM primitives `config_save()`/`config_init()` use (`ConfigHandler.cpp`), in its own EEPROM region (offset after `cfg`'s footprint), once the `true ||` bug is fixed.
- **New endpoints in `WiFiManager.cpp`**, following the existing per-manager pattern exactly:
  - `GET /followmanager/config` — returns current config as JSON (all §9 keys, resolved values — i.e. reflects grid→canonical expansion so the UI can show both views).
  - `POST /followmanager/config` — accepts JSON body with any subset of §9 keys, validates (including the §7.4 minimum-separation check server-side, not just client-side), updates the in-RAM struct, calls `config_save()`-equivalent to persist, and returns the resolved config back (so the UI can confirm what actually took effect after validation).
  - `GET /followmanager/status` — separate from config: live state for the panel to show while flying/bench-testing — current `PeerLock` state (`IDLE`/`ACQUIRING`/`LOCKED`/`LOCKED_HOLDING`), which peer id/name is locked (if any), gate active/inactive, and the last computed target (for the bench test in §11.1).
- **New web UI panel**, modeled on the existing `Settings()` component's structure in `html/main.js` but wired to the new endpoints (not the dead `/system/status` path): friendly-grid dropdowns (§7.3) as the primary editing surface with an "advanced" toggle to edit raw `FOLLOW_OFS_*_M`, trigger-mode selector, and a target-peer selector populated live from `/peermanager/status` (so the pilot can pick a specific visible peer by name instead of only `FIRST_ACTIVE`) — this doubles as the escape hatch referenced in §6.3's "user changes a setting" transition.
- **Resolution order:** compile-time `build_flags` values are the factory defaults used only to seed EEPROM the first time (or after a config reset); once the web UI has saved a value, the persisted value wins on every subsequent boot. This mirrors how `cfg`/`config_save()` would work correctly once §10.1's bug fix lands.

### 10.3 Non-goals for this section
- No auth on these endpoints, consistent with the rest of FF's local-AP-only web UI (`WiFi.softAP`, no credentials checked on other endpoints either) — not introducing a new security posture here.
- No remote/cloud config sync. Persistence is local EEPROM only, same trust boundary as the rest of FF.

---

## 11. Files / Modules to Change (feeds the plan)

1. **`src/lib/MSP/MSPManager.{h,cpp}`** — add `sendFollowWaypoint()` using the existing `msp_set_wp_t` + `MSP::command()` (§6.1, not a hand-rolled frame); add `MSP_ALTITUDE` polling + cached `local_altitude_cm()` (§5[B] — confirmed new, not reuse); extend the existing `getActiveModes()`-based state read to expose a "GCS NAV active" accessor (§5[C] — one-line addition to existing code); if using AUX trigger, add new `MSP_RC` polling + accessor.
2. **`src/lib/Peers/PeerManager.{h,cpp}`** — expose a lookup-by-id accessor (peer table is index-based today, not id-keyed — needed for §6.3's lock-by-id) and a `peer_is_stale(peer_t*)` helper using `peer->updated` (§8). No struct changes needed — `peer->gps.lat/lon/groundCourse/groundSpeed` and `peer->relalt` already carry what's needed (§5[A]).
3. **`follow` (new module, e.g. `src/lib/Follow/FollowManager.{h,cpp}`)** — `loop()` (§6.2), `PeerLock` state machine (§6.3), `follow_resolve_offset()`, `slotToLatLon()`, `follow_switch_active()`, `follow_target_sane()`, plus a `statusJson()`/config accessors for §10's endpoints.
4. **`src/main.cpp`** — register `FollowManager::getSingleton()->loop()` in the main `loop()` sequence, after `PeerManager`/`GNSSManager`/`MSPManager`, gated the same way `MSPManager` is (§3).
5. **`src/lib/ConfigHandler.cpp`** — fix the `if (true || ...)` bug (`ConfigHandler.cpp:37`) as a prerequisite for §10's persistence to work at all; this is a pre-existing defect, worth flagging to the team as possibly wanted/known before "fixing" it out from under other config.
6. **`src/lib/WiFi/WiFiManager.cpp`** — add `/followmanager/config` (GET+POST) and `/followmanager/status` (GET) endpoints (§10.2), following the existing per-manager handler pattern; optionally extend `/peermanager/spoof` or add a parametrized alternative (lat/lon/course, modeled on `/gnssmanager/spoof`'s param handling) to support §11.1's bench tests.
7. **`html/main.js`** (or a new component file) — new Follow config panel (§10.2), replacing/ignoring the vestigial `Settings()` component's dead save path.
8. **Config / target `.ini` files** — add all §9 keys as `build_flags` defaults (factory-default seed values for §10's first-boot EEPROM init).

---

## 12. Test & Acceptance

### 12.1 Bench (props OFF, before any flight)
Existing peer-spoofing is more limited than v1 assumed: `POST /peermanager/spoof` (`PeerManager::enableSpoofing`) exists and injects a **fixed synthetic ring of 5 peers** 100 m apart around self — it does not accept an arbitrary lat/lon/course per request. (There is no `/mspmanager/spoof` — that endpoint does not exist.) For repeatable per-preset geometry verification, either extend the spoof mechanism to accept lat/lon/course params (mirroring `/gnssmanager/spoof`'s existing param handling, `WiFiManager.cpp:111-127`) or verify geometry against the fixed 5-peer ring's known positions.

Verify:
1. Emitter fires **only** when the gate is active (switch/mode).
2. **Peer-lock behavior (§6.3):** with the 5-peer spoof ring active, enabling follow mode locks onto exactly one peer (the first with `id > 0`/not lost) and continues tracking only that id even as other peers update. Killing updates for the locked peer (while others keep updating) causes a transition to `LOCKED_HOLDING` (emission stops, no switch to another peer) — confirm via `/followmanager/status`. Restoring the same peer's updates resumes emission without changing which id is tracked. Toggling the follow switch off/on re-acquires (possibly a different first-detected peer).
3. For each named preset in §7.3, the computed WP#255 lands in the correct place relative to the locked peer's position and `groundCourse`. Read it back with `MSP_WP` (#254) and/or observe in the INAV Configurator / OSD.
4. Altitude reference is correct (target alt tracks follower-home-relative frame; vertical offset applies with correct sign).
5. Coordinate scaling verified: target is at the leader, **not** ~10× displaced — confirm the `* 10` conversion from `peer->gps.lat` (§5[A]) lands correctly.
6. Freshness guard: killing the spoof stops emission within `FOLLOW_PEER_TIMEOUT_MS`.
7. Geometry guards: a sub-minimum-separation config refuses to arm (both firmware-side and web-UI-side, §7.4); stationary-leader fallback behaves as configured — spoof the leader at ~1.5 m/s and ~3 m/s bracketing the `FOLLOW_MIN_COURSE_SPEED` default and confirm the fallback triggers/doesn't at the correct threshold (the cm/s-vs-m/s comparison in §7.5 is easy to get silently wrong).
8. Altitude accuracy: with two real GPS units (not spoofed) at a known, measured height difference, confirm the commanded `alt_cm` lands within `FOLLOW_MIN_VSEP_M` of the intended offset — quantifies the real-world GPS-vertical-error budget the 13 m default (§7.4) is meant to absorb; §12.1 item 4 above only checks `local_altitude_cm()` in isolation, not the combined leader+follower altitude math.
9. **Web UI config (§10):** editing a preset/offset/trigger-mode/target-peer in the web panel, saving, and rebooting the follower confirms the value persisted (survives reboot) rather than reverting to compile-time defaults. Confirm changing `FOLLOW_TARGET_PEER` while the gate is active forces a re-acquire per §6.3.

### 12.2 Flight (progressive, open area, big margins)
1. Trail slot, large horizontal gap, generous vertical separation, low speed. Manual-override switch tested first.
2. Confirm follower captures and holds the slot; confirm instant manual recovery.
3. Confirm peer-lock hold behavior in flight: with two peers airborne, verify the follower locks the first and does not jump to the second if the first's link briefly drops.
4. Only then reduce gaps / try lateral and stacked (overhead) slots.

### 12.3 Acceptance criteria
- Follower reliably captures and holds every configured named slot within a bounded position error, at the configured altitude offset.
- Losing the peer link or flipping the switch always returns control safely (hold → POSHOLD/manual) with no flyaway.
- No `MSP_SET_WP` is ever emitted with stale, out-of-bounds, or mis-scaled coordinates.
- With multiple peers visible, the follower always locks to exactly one (the first detected, or the explicitly configured id) and never auto-retargets to another on loss.
- All follow parameters are editable and persist via the web UI without a reflash.

---

## 13. Open Questions / To Confirm

Resolved by source review (kept here for traceability, not action items): PeerManager struct field names/scaling (§5[A]), whether MSPManager has a generic send helper (§6.1, yes — `MSP::command()`), whether MSP_RC/status polling exists (§5[C], partially — `getActiveModes()` reused), scheduler API (§3, none — flat self-gated `loop()`).

Still open:
- **`peer->id` reuse edge case (§6.3):** confirm via bench/multi-peer testing whether a dropped-and-reassigned LoRa slot can plausibly happen within a `LOCKED_HOLDING` window in practice, and whether the name-match mitigation is sufficient or a stronger identity check is needed.
- **Altitude-frame mixing (§6.2, §7.4):** `alt_cm` sums the follower's baro/GPS-fused home-relative estimate with a raw-GPS-only delta from the peer link, because the leader only broadcasts raw GPS telemetry (§1.3 rules out leader-side changes). `FOLLOW_MIN_VSEP_M`'s revised 13 m default is a mitigation (absorb ~8 m of worst-case GPS vertical error), not a fix to the underlying measurement. A future leader-side enhancement — broadcasting the leader's own `MSP_ALTITUDE`-derived home-relative altitude instead of relying on the raw-GPS delta — would remove this error source entirely, but is out of scope for this iteration per §1.3. Worth quantifying with real two-aircraft altitude testing (§12.1 item 8) before deciding whether it's worth pursuing.
- **EEPROM wear from frequent web UI saves (§10):** if the panel encourages live-tweaking-while-hovering (e.g. dragging a slider), debounce saves rather than writing on every change, to avoid excessive EEPROM write cycles.
- **Whether the `config_init()` `true ||` bug (§10.1) is intentional/known** — confirm with the team before removing it, in case something currently depends on config always resetting (e.g. a support workaround).
- Whether an existing FF branch/PR already targets a follow feature or the config-write endpoint independently (check the project's #development channel before building) — a repo-wide grep and recent branch/commit review found no such work in progress as of this spec's writing, but that can change.
