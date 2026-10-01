# FormationFlight — Follow Path-Tracking Mode — Engineering Spec

**Status:** Draft — not yet planned or implemented
**Target firmware:** FormationFlight (ESP32/ESP8266, PlatformIO/C++)
**Depends on:**
- [`2026-07-31-FollowMeOnInav.md`](2026-07-31-FollowMeOnInav.md) — the parent spec. This addendum reuses its canonical offset model (§7.3), geometry safety rules (§7.4), altitude floor (§7.6), and `FollowRuntimeConfig`/EEPROM persistence design (§10).
- [`2026-08-15-FollowRcAxisControl.md`](2026-08-15-FollowRcAxisControl.md) — this spec's validation rules interact with RC axis control's lateral-channel assignment (§5.1 below).

---

## 1. Purpose & Scope

### 1.1 Problem

Today's follow slot (`ofsLongM`/`ofsLatM`/`ofsVertM`, parent spec §7.3) is computed fresh every cycle as "the leader's **current** position, plus a fixed offset rotated by the leader's **current** course" (`slotToLatLon()`, `FollowManager.cpp:35-67`). There is no memory of where the leader has actually been. On a turn, the offset point swings the instant the leader's course changes — the follower cuts the corner rather than tracing the leader's actual ground track, which is visibly wrong for chase footage and unnecessarily tight for formation clearance through a turn.

### 1.2 Goal

Add a second follow track mode, **`PATH`**, that reconstructs the leader's recent ground track from received telemetry and computes the follower's target as a point on that recorded track — arc-length `|ofsLongM|` behind the leader's current position — instead of a straight-line offset from the leader's current position and course. A follower slotted directly behind the leader in `PATH` mode traces the same curve the leader flew, corners included.

### 1.3 In scope

- A follower-side breadcrumb buffer of the locked leader's recent telemetry (§3.1), built entirely from data FormationFlight already receives — no leader-side or protocol changes.
- An arc-length walk-back algorithm that resolves "the point on the recorded track `D` meters behind the leader's current position" (§3.2), including the leader's historical relative altitude at that point, so a follower slotted behind-and-above/below also retraces climbs and descents, not just the lat/lon track.
- A new `FollowTrackMode` config field (`LIVE` | `PATH`), live-editable and EEPROM-persisted the same way every other `FollowRuntimeConfig` field already is (§5).
- Two new `GNSSManager` static helpers — point-to-point distance and bearing — needed by the walk-back algorithm and exposing math (`distanceMeters()`/`courseDegrees()`, `GNSSManager.cpp:133-159`) that already exists in that file but isn't currently callable from outside it (§3.2).
- A new condition code for "the recorded track doesn't reach back far enough to cover the configured offset" (§4.3), following the existing `conditionFlagsGvarIndex` pattern (parent spec's OSD/GVAR addendum, `2026-08-13-FollowStatusOsdGvar.md`).

### 1.4 Out of scope (this iteration)

- **Lateral (`ofsLatM`) or ahead (`ofsLongM >= 0`) offsets in `PATH` mode.** `PATH` mode is restricted to `ofsLongM < 0`, `ofsLatM == 0` (§5.1). There is no recorded track ahead of the leader to trace, and a laterally-offset parallel curve through a corner is a different, geometrically inexact curve, not "the leader's exact path" — supporting it would silently misrepresent what this feature does. Deliberately excluded rather than approximated; see §10 for the reasoning against a "best-effort" lateral approximation.
- **Leader-side changes.** As with the parent spec (§1.3 there), the leader only ever broadcasts what it already does today. The breadcrumb buffer is reconstructed entirely from the follower's own received telemetry stream.
- **Persisting the breadcrumb buffer across reboot or across a peer re-lock.** It's RAM-only, rebuilt from scratch every time a peer is (re-)locked (§3.3) — there's no requirement that a follower resume mid-trace after a restart.
- **Any interaction with the follow-mode trigger/gate itself.** `PATH` mode only changes how the position target is computed while follow is already active — same non-goal framing as the RC axis control spec (§1.4 there).

### 1.5 Assumptions

- The follower FC is INAV, connected over MSP, the same link used for every other follow-mode data source.
- The leader's telemetry update rate and the LoRa TDMA cycle time are as described in the parent spec (§5[A], §8) — this spec's buffer sizing (§3.1) is chosen against those same assumptions, not a new transport.

---

## 2. Background

### 2.1 Why today's math cuts corners

`FollowManager::loop()` (`FollowManager.cpp:564-568`) calls `slotToLatLon(peer->gps.lat, peer->gps.lon, courseDeg, offset.longitudinal_m, offset.lateral_m)` every cycle. `slotToLatLon()` (`FollowManager.cpp:35-67`) rotates the configured offset by the leader's **current** `groundCourse` and projects it from the leader's **current** lat/lon via `GNSSManager::calculatePointAtDistance()`. Both inputs are single point-in-time values — there is no persisted "previous target" and no interpolation between cycles (confirmed: `FollowManager.h`/`.cpp` hold no leader-position history anywhere; `peer_t::gps_pre`/`gps_pre_updated`, `PeerManager.h:24-25`, are written by `RadioManager::receive()` but never read by `FollowManager`). The result: a `BEHIND` slot is always "X meters directly behind wherever the leader is pointed right now," which snaps instantly to the new bearing the moment the leader's course changes — the follower cuts across the inside of every turn instead of flying through the point the leader was actually at.

### 2.2 Why this is solvable without touching the leader or the radio protocol

The follower already receives a live, continuous stream of the leader's `lat`/`lon`/`relalt` (parent spec §5[A]) at up to the LoRa cycle rate. Nothing about "trace the leader's actual path" requires new data — it only requires the follower to **remember** the last several samples of what it already receives, instead of discarding everything but the newest one. This is analogous to the "boat wake" or "path-lock" technique used in real formation/chase flying: fly to where the lead aircraft *was*, not where it currently is, offset by however far back the desired following distance corresponds to along the track.

---

## 3. Breadcrumb Buffer & Path Walk-Back Algorithm

### 3.1 Buffer structure and lifecycle

New struct (`FollowManager.h`), one ring buffer owned by `FollowManager` itself (not `PeerManager` — this is Follow-specific state, no other consumer needs it):

```cpp
struct FollowPathSample {
    int32_t lat_1e6;     // peer->gps.lat verbatim (PeerManager.h's internal x1e6 representation)
    int32_t lon_1e6;     // peer->gps.lon verbatim
    int16_t relAltM;     // peer->relalt at capture time (leader - follower, raw-GPS delta)
    uint32_t timestampMs; // peer->updated at capture time
};
```

`FollowPathSample pathBuffer[FOLLOW_PATH_BUFFER_CAPACITY]` (new `FollowConfig.h` compile constant, default **64**) plus `uint16_t pathBufferHead`/`pathBufferCount` — a standard fixed-capacity ring buffer, oldest sample overwritten once full. At 14 bytes/entry, 64 entries is 896 bytes — negligible even on the ESP8266 targets this repo ships (`targets/*.ini`'s `esp82xx` bases).

**Push (`maybePushPathSample()`, new private method):** called from `loop()` immediately after `resolveLock()` returns a live peer, gated on `config.trackMode == FOLLOW_TRACK_PATH` (zero cost when the feature isn't in use, matching the `-1`-disables-by-default precedent RC axis control already established). Dedups on `peer->updated`: if the newest buffered sample's `timestampMs` already equals the current `peer->updated`, skip the push — `loop()` runs at `emitHz` (default 4 Hz), which can be faster than the leader's actual telemetry update rate, and pushing an unchanged reading would waste buffer capacity on zero-length segments.

**Reset (`resetPathBuffer()`, new private method, `pathBufferCount = 0`):** called at every point the buffer's previous contents would be invalid for the *next* thing it's asked to represent:
- `resolveLock()`'s `ACQUIRING → LOCKED` transition (`FollowManager.cpp:131-135`) — a freshly locked peer's history must start empty; splicing in a previous peer's (or a previous lock session's) trail would silently misrepresent the current leader's path.
- `forceReacquire()` (`FollowManager.cpp:170-175`) — same reasoning, since it clears `lockedId` and forces a fresh `ACQUIRING`.
- `loop()`'s gate-inactive branch (`FollowManager.cpp:547-555`), alongside the existing `lockedId = 0`/`haveValidCourse = false` reset.

**Not reset** on `LOCKED → LOCKED_HOLDING → LOCKED` (a brief peer staleness, same id resumes, parent spec §6.3) — the recorded history is still valid, the walk-back just spans a real gap in telemetry across the hold, which is the same "no continuity guarantee across a `LOCKED_HOLDING` gap" behavior the RC axis control spec already documents and accepts (`2026-08-15-FollowRcAxisControl.md` §11).

### 3.2 Walk-back algorithm

New `GNSSManager` static helpers (mirroring `calculatePointAtDistance()`'s existing static-method pattern, `GNSSManager.h:66`), thin wrappers promoting the file-local free functions that already implement this math (`GNSSManager.cpp:133-159`) to something callable outside that translation unit:

```cpp
static double GNSSManager::distanceBetween(GNSSLocation a, GNSSLocation b); // wraps distanceMeters()
static double GNSSManager::bearingBetween(GNSSLocation a, GNSSLocation b);  // wraps courseDegrees()
```

No behavior change to existing callers — `horizontalDistanceTo()`/`courseTo()` (`GNSSManager.cpp:161-171`) keep calling the same free functions with `this->getLocation()` as one argument; the new statics just expose the same two-arbitrary-points computation FollowManager needs for two *historical* leader points, neither of which is "self."

`FollowManager::resolvePathTarget()` (new private method), called from `loop()` only when `config.trackMode == FOLLOW_TRACK_PATH`:

```cpp
// Walks backward through pathBuffer from the newest sample, accumulating
// great-circle distance between consecutive samples, until `longitudinal_m`
// (always < 0 here — see §5.1's validation) worth of arc length is covered.
// Returns false only when the buffer has fewer than 2 samples (nothing to
// interpolate yet, e.g. immediately after a fresh lock).
bool FollowManager::resolvePathTarget(double longitudinal_m, FollowTarget *outTarget,
                                       int16_t *outRelAltM, bool *outInsufficientHistory);
```

Algorithm:
1. `remaining = fabs(longitudinal_m)`.
2. Starting from the newest buffered sample, walk toward the oldest one pair at a time. For each adjacent pair `(newer, older)`, compute `segLen = GNSSManager::distanceBetween(newer, older)`.
3. If `segLen >= remaining`: the target lies on this segment. Interpolate — `bearing = GNSSManager::bearingBetween(newer, older)`, then `interp = GNSSManager::calculatePointAtDistance(newer, remaining, bearing)` (reusing the same proven great-circle projection `slotToLatLon()` already uses, rather than a flat-earth lat/lon lerp — consistent with that function's existing comment on why it uses `calculatePointAtDistance()` instead of hand-rolling one). `relAltM` is linearly interpolated between the two samples' `relAltM` by the same fraction (`remaining / segLen`). Return `true`, `*outInsufficientHistory = false`.
4. Otherwise: `remaining -= segLen`, advance to the next older pair, repeat.
5. If the walk reaches the oldest buffered sample without `remaining` reaching zero (the recorded track doesn't span the configured offset — buffer too small for the leader's speed/telemetry rate combination, or the peer was only just locked): **clamp** to the oldest available sample verbatim rather than extrapolating past it, return `true`, `*outInsufficientHistory = true`. Clamping (not rejecting) mirrors the parent spec's altitude-floor philosophy (§7.6 there: "clamp, don't reject... keep tracking... rather than stop emitting entirely") — the follower still gets a real, on-track point, just closer than configured, with the shortfall surfaced via a condition code (§4.3) instead of silently swallowed.

### 3.3 loop() integration

```cpp
FollowOffset offset = resolveOffset();
double courseDeg = resolveCourseDeg(peer); // still needed for heading modes (§4.2) regardless of trackMode

FollowTarget target;
int16_t effectiveRelAltM = peer->relalt; // LIVE-mode default
bool pathHistoryInsufficient = false;

if (config.trackMode == FOLLOW_TRACK_PATH)
{
    maybePushPathSample(peer);
    FollowTarget pathTarget;
    int16_t pathRelAltM;
    if (!resolvePathTarget(offset.longitudinal_m, &pathTarget, &pathRelAltM, &pathHistoryInsufficient))
    {
        // Fewer than 2 samples yet -- hold rather than emit a target
        // computed from a single point, which would just be this cycle's
        // live position and defeat the point of PATH mode on the very
        // first cycles after lock.
        bail(FOLLOW_CONDITION_PATH_HISTORY_INSUFFICIENT);
        return;
    }
    target = pathTarget;
    effectiveRelAltM = pathRelAltM;
}
else
{
    target = slotToLatLon(peer->gps.lat, peer->gps.lon, courseDeg,
                           offset.longitudinal_m, offset.lateral_m);
}
```

`ofsLatM` is not passed to `slotToLatLon()` at all in the `PATH` branch — §5.1's validation already guarantees it's `0` and RC lateral control is disabled whenever `trackMode == PATH`, so there is no lateral rotation to apply; the walked-back point *is* the target.

The altitude sum (`FollowManager.cpp:576-578`) changes its second term from `peer->relalt` to `effectiveRelAltM`:

```cpp
double altCmD = (double)msp->local_altitude_cm()
               + (double)effectiveRelAltM * 100.0
               + offset.vertical_m * 100.0;
```

Everything downstream of `target`/`altCm` — the altitude floor clamp (§7.6), `offsetGeometrySane()`/`targetTooFar()` checks, heading resolution, the speed autothrottle, `sendFollowWaypoint()` — is unchanged; `PATH` mode only changes how `target` and the altitude's leader-relative term are produced, not anything consuming them.

---

## 4. Interaction With Existing Follow Machinery

### 4.1 RC axis control (`2026-08-15-FollowRcAxisControl.md`)

Longitudinal and vertical RC scaling (`resolveOffset()`, `FollowManager.cpp:477-493`) are unaffected — they still scale `ofsLongM`/`ofsVertM` before either code path runs; `PATH` mode just changes what origin that scaled longitudinal value is measured from (a walked-back path point instead of the live point). The two-layer geometry safety net (Layer 1 `offsetGeometrySane()`, Layer 2 sign-lock) operates entirely in offset-triple space and has no dependency on which track mode is active, so no changes are needed there.

Lateral RC scaling is incompatible with `PATH` mode by definition (§1.3) — see §5.1 for the resulting validation rule (`rcLatChannel` must be disabled whenever `trackMode == PATH`).

### 4.2 Heading modes (parent spec §7.7)

`resolveHeadingDeg()` is called with the same `courseDeg` (the leader's **live** course, from `resolveCourseDeg()`) regardless of `trackMode`. Nose orientation is a separate decision from position offset — a `PATH`-mode follower tracing the leader's turn still points wherever `FOLLOW_HEADING_MODE` says to point (e.g. `POINT_LEADER` still points at the leader's live position, not the walked-back point). No changes needed.

### 4.3 Condition codes (parent spec §7.6, OSD/GVAR addendum, RC axis control §4.5)

New code, appended per `FollowConditionCode`'s existing ordering convention (`FollowManager.h:16-24`: "prefer appending future conditions... in priority order rather than renumbering"):

```cpp
FOLLOW_CONDITION_PATH_HISTORY_INSUFFICIENT = 4, // PATH mode's walk-back exhausted the buffer
```

Raised (via the existing `raiseCondition()` lambda, `FollowManager.cpp:614-616`) whenever `resolvePathTarget()` sets `*outInsufficientHistory = true`, and also covers the "buffer has fewer than 2 samples" bail-out case in §3.3. Ranked above the existing codes (highest priority) since it's the most immediately actionable for a `PATH`-mode pilot — "your configured following distance isn't actually being achieved" is a bigger deal for this mode specifically than a floor clamp or an RC freeze would be while `PATH` is engaged.

### 4.4 Altitude floor (parent spec §7.6)

Unchanged mechanism — still clamps the final summed `alt_cm`, still applied after all terms (including `PATH` mode's `effectiveRelAltM`) are summed. No new interaction to design; `effectiveRelAltM` simply substitutes for `peer->relalt` as an input to the same sum.

### 4.5 Speed autothrottle (`2026-08-28-FollowSpeedAutothrottle.md`)

`resolveTargetSpeedCmS()`/`resolveAlongTrackErrorM()` (`FollowManager.cpp:278-307`) consume `target` and `courseDeg` exactly as produced above — no changes needed, since by the time these run, `target` is already resolved regardless of which track mode produced it. One nuance worth confirming during implementation, not a design change: the along-track error is still measured relative to the leader's **live** course (`courseDeg`), which is a reasonable frame for "how far is the follower from its slot along the direction of current travel" even in `PATH` mode, since the follower's own closing motion isn't itself path-constrained — only the *target point* is.

---

## 5. Configuration Model

New `FollowTrackMode` enum (`FollowConfig.h`, alongside `FollowHeadingMode`):

```cpp
enum FollowTrackMode {
    FOLLOW_TRACK_LIVE = 0, // today's behavior: live leader position + live course (default)
    FOLLOW_TRACK_PATH = 1, // walk back along the leader's recorded track
};
```

`FOLLOW_TRACK_MODE` compile-time default (`#ifndef`-guarded like every other `FollowConfig.h` key), default `FOLLOW_TRACK_LIVE` — opt-in, matches every other feature-flag default in this file.

`FOLLOW_PATH_BUFFER_CAPACITY` compile-time constant, default `64` (§3.1). Deliberately **not** runtime-editable (unlike every other `FollowRuntimeConfig` field) — it sizes a fixed-size array member of `FollowManager`, so changing it is a reflash, not a web UI edit, the same category as `NODES_MAX`/`NAME_LENGTH` (`PeerManager.h:5-6`).

New `FollowRuntimeConfig` field (`FollowManager.h:53-121`):

| Field | Type | Default | Meaning |
|---|---|---|---|
| `trackMode` | `FollowTrackMode` | `FOLLOW_TRACK_MODE` | `LIVE` (today's behavior) or `PATH` (this spec) |

New `FollowEepromRecord` field: `FollowTrackMode trackMode;` — `DIRECT` field (already an integer-backed enum type, no `double`→`int16_t` narrowing needed, same treatment as `headingMode`). Bump `FOLLOW_EEPROM_VERSION` from `6` to `7` (same mechanism every prior field addition used).

### 5.1 Validation (`applyConfig()`, `FollowManager.cpp:903-1057`)

New checks, added alongside the existing offset-geometry check (`FollowManager.cpp:1039-1043`):

```cpp
if (newConfig.trackMode == FOLLOW_TRACK_PATH)
{
    if (newConfig.ofsLongM >= 0)
    {
        *errMsg = "trackMode PATH requires ofsLongM < 0 (behind the leader) -- "
                  "there is no recorded path ahead of the leader to trace";
        return false;
    }
    if (newConfig.ofsLatM != 0.0)
    {
        *errMsg = "trackMode PATH requires ofsLatM == 0 -- a laterally offset "
                  "curve through a corner is not the leader's exact path";
        return false;
    }
    if (newConfig.rcLatChannel != -1)
    {
        *errMsg = "trackMode PATH requires rcLatChannel disabled -- RC could "
                  "otherwise drive the lateral offset away from 0 in flight";
        return false;
    }
}
```

Same rationale as every other `applyConfig()` check: reject at config-write time so an invalid combination can never reach `loop()`'s runtime path, and enforce it server-side even though `html/follow-logic.js`'s `validateConfig()` (§7) also blocks it client-side.

`FOLLOW_CONFIG_DIRECT_FIELDS`/`FOLLOW_CONFIG_ROUNDED_FIELDS` (`FollowManager.cpp:871-882`) — `trackMode` is handled by hand in `configJson()`/`toEepromRecord()`/`fromEepromRecord()`, the same way `headingMode` already is (reported as a name string in JSON, carried through EEPROM as its raw enum value), not added to either X-macro list.

---

## 6. Status Endpoint Additions

`GET /followmanager/status` (`statusJson()`, `FollowManager.cpp:697-743`) gains, populated only while `trackMode == PATH` (mirrors the existing `haveLastTarget`-gated pattern):

- `pathBufferCount: int` — samples currently held (`0`..`FOLLOW_PATH_BUFFER_CAPACITY`).
- `pathBufferSpanM: double` — total arc length currently spanned by the buffer (sum of consecutive-sample distances) — lets a pilot see, before or during flight, whether the buffer actually covers their configured `|ofsLongM|` at the leader's current speed/telemetry rate.
- `pathHistoryInsufficient: bool` — mirrors the last cycle's `FOLLOW_CONDITION_PATH_HISTORY_INSUFFICIENT` state, exposed directly so the web panel can show it without a condition GVAR configured (same reasoning as `rcSlotFrozen` in the RC axis control spec, §7 there).

`GET /followmanager/config` (`configJson()`) gains `trackMode` as a name string (`"LIVE"`/`"PATH"`), following the existing `headingMode`/`triggerMode` pattern exactly.

---

## 7. Web UI (`html/follow.js`)

- A "Track Mode" `<select>` (`LIVE` / `PATH`) in the existing geometry panel, alongside the offset/grid fields.
- When `PATH` is selected: grey out (or hide, matching whichever pattern the existing grid/advanced-offset toggle uses) the lateral offset field and the RC lateral-channel dropdown, and surface §5.1's constraint as static copy ("Path mode traces directly behind the leader only — no lateral offset") rather than letting the pilot discover it only via a rejected save.
- Client-side `validateConfig()` addition mirroring §5.1's server-side checks exactly (same precedent as every other firmware-side rule already duplicated client-side in this file) — block Save with an inline error rather than round-tripping to the server to find out.
- Status panel: while `trackMode == PATH`, show `pathBufferCount`/`pathBufferSpanM` next to the existing `lastTarget` readout, and an inline warning when `pathHistoryInsufficient` is true (same visual treatment as the existing `rcSlotFrozen` warning).

---

## 8. Files / Modules to Change

1. **`src/lib/GNSS/GNSSManager.h`/`.cpp`** — add `static double distanceBetween(GNSSLocation, GNSSLocation)` and `static double bearingBetween(GNSSLocation, GNSSLocation)`, thin wrappers over the existing file-local `distanceMeters()`/`courseDegrees()` (`GNSSManager.cpp:133-159`) — no behavior change to any existing caller (§3.2).
2. **`src/lib/Follow/FollowConfig.h`** — `FollowTrackMode` enum, `FOLLOW_TRACK_MODE` default, `FOLLOW_PATH_BUFFER_CAPACITY` constant (§5).
3. **`src/lib/Follow/FollowManager.h`** — `FollowPathSample` struct, ring buffer members (`pathBuffer`/`pathBufferHead`/`pathBufferCount`), `trackMode` field on `FollowRuntimeConfig`/`FollowEepromRecord`, `FOLLOW_CONDITION_PATH_HISTORY_INSUFFICIENT` (§4.3), new private methods `maybePushPathSample()`, `resetPathBuffer()`, `resolvePathTarget()`.
4. **`src/lib/Follow/FollowManager.cpp`** — `resolveLock()` gains `resetPathBuffer()` calls at the lock-acquire and reset points (§3.1); `loop()` gains the `trackMode` branch (§3.3) and the `effectiveRelAltM` substitution in the altitude sum; `applyConfig()` gains §5.1's validation; `configJson()`/`toEepromRecord()`/`fromEepromRecord()`/`statusJson()` gain `trackMode` (and, for `statusJson()`, `pathBufferCount`/`pathBufferSpanM`/`pathHistoryInsufficient`) by hand, same treatment as `headingMode`.
5. **`html/follow.js`** — Track Mode dropdown, conditional lateral-field disabling, client-side validation, status panel additions (§7).
6. **`test/test_follow_native/`** — new `test_path_tracking.cpp`, mirroring `test_slot_geometry.cpp`'s pure-function-test structure: feed `resolvePathTarget()` a synthetic breadcrumb trail with a deliberate corner (an L-shape or the same hexagon shape `PeerManager::spoofPeerHexPath()` already generates for bench testing) and assert the resolved target lies **on** the recorded polyline at the correct arc-length offset, not on the straight-line chord `slotToLatLon()` would have produced — this is the test that actually proves the corner-cutting problem is fixed. Also cover: buffer-exhaustion clamping (`pathHistoryInsufficient`), the `< 2` samples hold case, and `applyConfig()`'s new rejection rules (§5.1).
7. **Config/target `.ini` files** — add `FOLLOW_TRACK_MODE` as a `build_flags` default (`LIVE`), matching every other §5/parent-spec §9 key's pattern.

---

## 9. Test & Acceptance

### 9.1 Bench (props off)

1. **Corner-tracing, the core acceptance test:** `POST /peermanager/spoof` with `spoofPeerHexPath()` (already exists, `PeerManager.cpp` — sends a spoofed peer around a closed hexagon, i.e. a path with six corners). Engage follow with `trackMode = PATH`, `ofsLongM` set to some value well inside one hexagon edge's length. Poll `/followmanager/status`'s `lastTarget` across a full lap and confirm the commanded target traces the hexagon's edges and vertices — i.e. it stays close to the polyline the spoofed peer is flying, not the straight chord `LIVE` mode would produce through each vertex. Repeat with `trackMode = LIVE` against the same spoofed path as a side-by-side control, confirming the corner-cutting behavior is visibly present there and absent in `PATH` mode.
2. **Buffer growth/dedup:** confirm `pathBufferCount`/`pathBufferSpanM` grow as the spoofed peer moves and do **not** grow while its telemetry timestamp is unchanged (dedup, §3.1).
3. **Insufficient-history clamp:** configure an `ofsLongM` larger than the spoofed path's total available recorded span (e.g. right after engaging, before the buffer has filled); confirm `pathHistoryInsufficient` is reported and the target clamps to the oldest available sample rather than extrapolating past it or reverting to `LIVE`-style projection.
4. **Buffer reset on re-lock:** lock peer A in `PATH` mode, let its buffer fill, then force a re-acquire onto a different peer B (`FOLLOW_TARGET_PEER` change, parent spec §6.3) and confirm the buffer is empty immediately after (`pathBufferCount == 0`), not spliced with peer A's trail.
5. **Vertical tracing:** spoof a peer on a path with a nonzero altitude profile (e.g. `spoofPeerHexPath()`'s existing climb/descent behavior) and confirm the commanded altitude in `PATH` mode reflects the leader's **historical** relative altitude at the walked-back point, not its live one — verifiable by comparing against the same run's `LIVE`-mode altitude at a moment the leader's live and historical altitudes clearly differ (e.g. mid-climb).
6. **Validation rejections (§5.1):** confirm `applyConfig()`/`POST /followmanager/config` rejects `trackMode = PATH` combined with `ofsLongM >= 0`, `ofsLatM != 0`, or `rcLatChannel != -1`, each with a distinct error message; confirm the web UI's client-side check blocks Save the same way before ever reaching the server.

### 9.2 Flight (progressive, open area, big margins)

1. Large `BEHIND` gap, generous vertical separation, `PATH` mode, leader flying gentle, wide turns first.
2. Confirm the follower visibly traces the leader's turn radius rather than cutting inside it; confirm instant manual recovery via the existing override switch.
3. Progress to tighter turns / a real (not spoofed) multi-leg course only after step 2 is confirmed safe.

### 9.3 Acceptance criteria

- With `trackMode = PATH` and a `BEHIND`-only slot, the follower's commanded target measurably traces the leader's recorded ground track through a turn, rather than the straight-line-to-current-offset chord `LIVE` mode produces — demonstrated both on the bench (§9.1 item 1) and in flight (§9.2).
- `PATH` mode never commands a target computed from fewer than 2 recorded samples (holds instead, §3.3).
- A `PATH`-mode config with `ofsLongM >= 0`, `ofsLatM != 0`, or an assigned `rcLatChannel` is always rejected, both firmware-side and web-UI-side.
- Insufficient recorded history degrades to a clamp at the oldest available point plus a reported condition code — never a silent revert to `LIVE`-mode behavior and never an extrapolation past recorded data.
- All other follow behavior (heading modes, altitude floor, RC longitudinal/vertical scaling, speed autothrottle, peer-lock semantics) is unchanged when `trackMode = LIVE` (the default) — this feature is strictly additive/opt-in.

---

## 10. Open Questions

- **Buffer capacity vs. configured offset/leader speed:** `FOLLOW_PATH_BUFFER_CAPACITY = 64` is a reasonable-looking default but hasn't been validated against a real worst case (fast leader + slow telemetry update rate + a large configured `|ofsLongM|`). Worth computing the actual worst-case span (`capacity × max plausible inter-sample distance`) against realistic LoRa cycle times (parent spec §8) before shipping, and deciding whether the constant needs to be larger, or whether `pathBufferSpanM` in the status endpoint (§6) is sufficient for a pilot to self-diagnose "increase distance between telemetry updates isn't possible, so keep the configured offset within the observed span."
- **Whether a "best-effort" lateral approximation is worth adding later.** This iteration rejects any nonzero `ofsLatM` in `PATH` mode outright (§1.3, §5.1) rather than approximating a parallel offset curve via the local track bearing at the walked-back point. That approximation is mechanically simple to add (the same rotation `slotToLatLon()` already does, just anchored to a path point instead of a live one) if a future revision decides "approximately traces the path, offset to the side" is more useful in practice than "exactly traces the path, directly behind only." Deliberately deferred rather than speculatively designed here, consistent with this spec's framing that an inexact lateral offset shouldn't be silently presented as "the exact path."
- **Great-circle vs. straight-hop interpolation within a segment (§3.2 step 3).** The walk-back interpolates within a single buffered segment via `calculatePointAtDistance()` from the newer endpoint, which is exact great-circle math for that one hop but doesn't attempt to spline/smooth across multiple segments — the reconstructed track is a polyline (straight hops between consecutive telemetry samples), not a smoothed curve. This should be a non-issue at typical telemetry update rates (samples close enough together that the polyline closely approximates the leader's true continuous path) but hasn't been bench-verified against a genuinely sparse telemetry scenario (e.g. many peers sharing the LoRa cycle, §8 of the parent spec).
- **Interaction with a peer-lock ID-reuse event (parent spec §6.3's caveat).** If `LOCKED_HOLDING`'s name-mismatch check detects the LoRa slot id was reassigned to a different aircraft, `resolveLock()` already falls through to a lost-lock state; confirm the follow-up `IDLE`→`ACQUIRING`→`LOCKED` sequence's buffer reset (§3.1) actually fires in that specific path during implementation, since it's a less obvious trigger than the ordinary gate-off/`forceReacquire()` cases.
