# FormationFlight — Follow Position/Velocity Filtering — Engineering Spec

**Status:** Draft — not yet planned or implemented
**Target firmware:** FormationFlight (ESP32/ESP8266, PlatformIO/C++)
**Depends on:**
- [`2026-07-31-FollowMeOnInav.md`](2026-07-31-FollowMeOnInav.md) — the parent spec. This addendum reuses its `FollowRuntimeConfig`/EEPROM persistence design (§10) and doesn't change the offset/geometry/altitude-floor model.
- [`2026-09-08-FollowPathTracking.md`](2026-09-08-FollowPathTracking.md) — that spec's breadcrumb buffer (§3.1 there) should record the *filtered* leader samples this spec produces, not raw ones; this spec's capture/emit decoupling (§4 below) replaces the loop() gating that spec's own §3.1 sketch assumed. This spec also reuses the two new `GNSSManager` static helpers that spec introduces (`distanceBetween()`/`bearingBetween()`, its §3.2) — implement whichever of the two specs lands first with those statics, the other reuses them. Independent otherwise: this spec's filtering benefits `LIVE` mode as much as `PATH` mode (§2.3).
- [`2026-08-28-FollowSpeedAutothrottle.md`](2026-08-28-FollowSpeedAutothrottle.md) — the follower-side filter (§3.4) feeds cleaner inputs into that feature's along-track error computation; no change to the autothrottle's own logic.

---

## 1. Purpose & Scope

### 1.1 Problem

GPS position fixes are noisy — ordinary receiver scatter, not gross error — for both the leader (received over the radio link) and the follower (its own onboard/FC-attached GNSS). Today `FollowManager` consumes both raw, every cycle: `LIVE` mode's `slotToLatLon(peer->gps.lat, peer->gps.lon, courseDeg, ...)` (`FollowManager.cpp:567-568`) projects directly from the leader's current raw lat/lon rotated by its raw `groundCourse` (`resolveCourseDeg()`, `FollowManager.cpp:177-202`), and the follower's own position is read fresh, unfiltered, every time `IFollowGnss::horizontalDistanceTo()`/`courseTo()` is called (`GNSSManager.cpp:161-171`: both call `this->getLocation()` on every invocation). The visible result is a commanded target that jitters cycle to cycle even when the leader is flying a physically smooth line, and — since `LIVE` mode's rotation angle comes from the same noisy `groundCourse` field — a target that can also swing directionally on course jitter alone, independent of any real leader maneuver.

### 1.2 Goal

Smooth both the leader's and the follower's own position/velocity with one lightweight recursive filter design, applied **upstream of whichever track mode consumes it** — so `LIVE` mode (today's only mode) and `PATH` mode (the addendum spec, not yet implemented) both benefit from the same noise rejection, rather than filtering being bolted on as a `PATH`-only feature.

### 1.3 In scope

- One position/velocity filter instance tracking the locked leader, fed from received peer telemetry (§3.1-§3.2).
- One position/velocity filter instance tracking the follower's own position, fed from a new `IFollowGnss::getSelfLocation()` seam (§3.4).
- A capture/emit decoupling of `FollowManager::loop()` (§4) so both filters update every `loop()` call — bounded only by however often new telemetry/GNSS samples actually arrive — instead of being throttled to `config.emitHz` alongside the rest of the follow computation.
- Two new `FollowRuntimeConfig` fields: one enable flag, one single tuning knob, applied identically to both filter instances (§5) — deliberately not platform-aware (multirotor vs. fixed-wing) in this iteration; see §1.4 and §7.
- `LIVE` mode's `slotToLatLon()` call site, `resolveCourseDeg()`, `resolveHeadingDeg()`'s `POINT_LEADER` branch, `horizontalOffsetM()`, and `targetTooFar()` all switched to read the relevant filtered estimate instead of a raw sample (§3.2, §3.4).

### 1.4 Out of scope (this iteration)

- **Any leader-side or protocol change.** The filter runs entirely off data the follower already receives today — same non-goal framing as both specs this one depends on.
- **Per-platform (multirotor vs. fixed-wing) filter tuning.** Considered (see conversation leading to this spec) and deliberately deferred: a single conservative gain, with the existing `minCourseSpeed` fallback still underneath it as a hover/near-zero-speed safety net (§3.3), is simpler and was judged adequate for this iteration. The follower's own mixer platform type is in fact already known for free (`IFollowMsp::getPlatformType()`, used today to gate the fixed-wing-only speed autothrottle) — a follow-up could branch the follower-side gain on it cheaply — but this iteration uses one gain for both the leader-side and follower-side filter, no platform branching anywhere.
- **A pilot-supplied "expected leader platform" hint config field.** Explicitly rejected for this iteration in favor of the single-gain approach above — the leader's platform type isn't in the radio packet (`air_type0_t`, §1.5) and adding a config field plus a leader-side behavior branch wasn't judged worth it for the marginal tuning improvement. May be revisited if bench/flight testing shows the single gain performs poorly for one airframe class.
- **Replacing `minCourseSpeed`'s hold-last-known-course fallback.** Kept exactly as-is, sitting underneath the filter (§3.3) — this spec changes what feeds it, not its own logic.
- **A windowed/moving-average filter, or any raw-sample history buffer for this feature specifically.** The filter design here is recursive: fixed-size state, no history array (§3.1, §6) — distinct from `PATH` mode's breadcrumb ring buffer, which is a separate, already-speced feature this filter's output feeds into.
- **Filtering used anywhere outside `FollowManager`.** `PeerManager`'s per-peer `distance`/`direction` fields (`PeerManager.cpp:180-181`, used for the peer table and OLED display) also call `GNSSManager::horizontalDistanceTo()`/`courseTo()`, but this spec does **not** touch those shared `GNSSManager` methods — see §3.4 for why, and the blast-radius reasoning behind keeping both filter instances local to `FollowManager`.

### 1.5 Assumptions

- Leader telemetry arrives via the existing air packet (`RadioManager.cpp`'s `air_type0_t`): `lat`/`lon`/`alt` update on every received packet, but `groundCourse`/`groundSpeed` only update once every 5 packets — they're carried in a rotating `extra_value` slot shared with the peer's name characters (`extra_type` cycles 0-4, `RadioManager.cpp:40-64`). So the leader's reported course/speed are both noisier *and* significantly staler than its position. This is the concrete, codebase-specific reason (not just a general GPS-quality argument) that §3.2's filter derives the leader's velocity internally from consecutive position samples, rather than reading `peer->gps.groundCourse`/`groundSpeed` directly.
- The follower's own `GNSSLocation.groundCourse`/`groundSpeed` (`GNSSManager::getLocation()`) update at whatever rate the connected GNSS provider natively runs at (e.g. `MSP_GNSS` polls every `UPDATE_LOOP_DELAY_MS` = 100ms, `GNSS/MSP_GNSS.h`) — not subject to the leader's 1-in-5 rotation, but still GPS-derived and still unreliable near zero groundspeed, so it's filtered the same way for consistency (§3.4).

---

## 2. Background

### 2.1 Why a recursive filter, not a windowed one

A windowed filter (moving average, median-of-N) needs to store N raw samples — RAM scales with window size, and a wider window (more noise rejection) directly costs more lag. A recursive filter — alpha-beta, the constant-velocity-model family used here — carries only its *current* position+velocity estimate forward; each new measurement updates that estimate and is then discarded. No history array, fixed-size state regardless of how far back "smooth" needs to reach. This also happens to be the answer to the earlier RAM question: state is a handful of doubles per axis, independent of buffer capacity or flight duration (§6).

### 2.2 Why velocity is filter *state*, not a filter *input*

The naive approach — feed the leader's reported `groundSpeed`/`groundCourse` into the filter as a velocity measurement — has two problems described in §1.5: that field is stale (updates 1-in-5 packets) and, more fundamentally, if it were ever derived by differencing two noisy position fixes rather than a genuine Doppler measurement, using it as a filter input would just reinject amplified position noise (differentiation is a high-pass operation). The alpha-beta design sidesteps this entirely: velocity is *estimated internally*, from the residual between predicted and measured position at each update (§3.1's `update()`). The filter never needs to trust an external speed/course reading at all — it only ever measures position.

### 2.3 Why this isn't `PATH`-mode-specific

`PATH` mode (the addendum spec) solves corner-cutting — a **geometry/lag** problem: `LIVE` mode always projects from the leader's *current* position+course, so the target snaps the instant the leader turns, even with perfectly noise-free GPS. Filtering solves **noise** — jitter in the position/course readings themselves — which exists regardless of which track mode is active, because both `LIVE`'s live point and `PATH`'s walked-back point are ultimately built from the same raw `peer->gps` stream. Concretely: `LIVE` mode's `slotToLatLon()` call (`FollowManager.cpp:567-568`) reads raw `peer->gps.lat`/`lon` and a raw-`groundCourse`-derived `courseDeg` **today**, with no dependency on `PATH` mode existing at all — so filtering is a strict improvement to the mode already shipped, not just infrastructure for a future one.

---

## 3. Filter Design

### 3.1 State & algorithm

One filter instance covers one tracked point (the leader, or the follower itself) as three independent 1D alpha-beta channels — north, east (a local flat-earth tangent-plane projection, not raw lat/lon degrees — see below for why), and altitude:

```cpp
// One recursive position+velocity channel. No history -- see SS2.1.
struct FollowFilterAxis1D {
    double position = 0.0;       // filtered estimate (meters, local frame)
    double velocity = 0.0;       // filtered estimate (m/s)
    unsigned long lastUpdateMs = 0;
    bool initialized = false;
};

struct FollowPositionFilter {
    GNSSLocation origin{};        // local tangent-plane anchor -- set once, SS3.1
    bool haveOrigin = false;
    FollowFilterAxis1D north;     // meters north of origin
    FollowFilterAxis1D east;      // meters east of origin
    FollowFilterAxis1D alt;       // meters (relalt for the leader instance, absolute for the follower instance)
};
```

Per-channel update (the same function, called three times per filter update — once for north, east, and altitude):

```cpp
// alpha, beta in (0, 1], derived from config.positionFilterStrengthPct (SS5).
void updateAxis1D(FollowFilterAxis1D *axis, double measurement, unsigned long nowMs, double alpha, double beta)
{
    if (!axis->initialized)
    {
        axis->position = measurement;
        axis->velocity = 0.0;
        axis->lastUpdateMs = nowMs;
        axis->initialized = true;
        return;
    }
    double dtS = (nowMs - axis->lastUpdateMs) / 1000.0;
    if (dtS <= 0.0) { return; } // duplicate/out-of-order timestamp -- nothing to do

    double predicted = axis->position + axis->velocity * dtS;
    double residual = measurement - predicted;
    axis->position = predicted + alpha * residual;
    axis->velocity = axis->velocity + (beta / dtS) * residual;
    axis->lastUpdateMs = nowMs;
}
```

**Why a local north/east tangent plane, not raw lat/lon degrees:** a longitude degree's length in meters shrinks by `cos(latitude)` — filtering raw lat/lon independently would give a `velocity` state with no consistent meaning in meters/second, making a derived course (`atan2`) meaningless. Instead, `FollowPositionFilter::origin` is set once (the *first* sample this filter instance sees after a reset, per §3.3/§3.4's reset points) via a flat-earth equirectangular projection: `northM = (lat - origin.lat) * metersPerDegLat`, `eastM = (lon - origin.lon) * metersPerDegLon(origin.lat)` (`metersPerDegLon` scaled by `cos(origin.lat)`, computed once when the origin is set). Error from the flat-earth approximation is negligible at the distances involved here (tens of meters of GPS scatter, `maxTargetDistM` on the order of tens of meters) — see §7 for the long-flight/large-loiter caveat this simplification defers.

Reading the filter back out (used by both `LIVE` mode and, later, `PATH` mode's breadcrumb push):

```cpp
GNSSLocation filteredLocation(const FollowPositionFilter &f);   // re-projects north/east through origin back to lat/lon
double filteredCourseDeg(const FollowPositionFilter &f);        // atan2(east.velocity, north.velocity), wrapped [0,360)
double filteredSpeedMps(const FollowPositionFilter &f);         // hypot(north.velocity, east.velocity)
```

### 3.2 Leader-side filter

One `FollowPositionFilter leaderFilter` member on `FollowManager`, updated whenever a new leader sample arrives — dedup'd on `peer->updated` (same timestamp-equality check `PATH` mode's `maybePushPathSample()` already needs, so both consumers share one dedup point once `PATH` mode exists) — feeding `peer->gps.lat`/`lon` (converted from x1e6, same conversion `slotToLatLon()` already does) and `peer->relalt` into `updatePositionFilter()`.

Reset (`resetLeaderFilter()`, `leaderFilter = FollowPositionFilter{}`) at exactly the points `PATH` mode's breadcrumb buffer also needs resetting (that spec's §3.1) — a freshly (re-)locked peer's filter state must start fresh, or the filter's first few estimates after a peer swap would be a weighted blend of the old and new leader's positions:
- `resolveLock()`'s `ACQUIRING → LOCKED` transition (`FollowManager.cpp:131-134`).
- `forceReacquire()` (`FollowManager.cpp:170-174`).
- `loop()`'s gate-inactive branch (`FollowManager.cpp:549-554`, alongside the existing `lockedId = 0`/`haveValidCourse = false` reset — see §4 for this branch's own restructuring).

**Consumers**, both switched from raw to filtered:
- `LIVE` mode's `slotToLatLon()` call (`FollowManager.cpp:567-568`) — reads `filteredLocation(leaderFilter)` instead of `peer->gps.lat`/`lon`.
- `resolveCourseDeg()` (`FollowManager.cpp:177-202`) — its `minCourseSpeed` comparison and `lastValidCourseDeg` both switch from `peer->gps.groundSpeed`/`groundCourse` to `filteredSpeedMps(leaderFilter)`/`filteredCourseDeg(leaderFilter)` (§3.3).
- The altitude sum's `peer->relalt` term (`FollowManager.cpp:576-578`) becomes `leaderFilter.alt.position`.

### 3.3 Interaction with `minCourseSpeed`

`resolveCourseDeg()`'s existing hold-last-known-course fallback (`FollowManager.cpp:185-201`: below `minCourseSpeed`, hold `lastValidCourseDeg` rather than following GPS course jitter while stationary) is kept exactly as-is — this is the hover/near-zero-groundspeed safety net that matters far more for multirotor leaders (which can genuinely hover) than fixed-wing ones (§1.4). Only its *inputs* change: the comparison and the held/reported value both come from the leader filter's own `filteredSpeedMps()`/`filteredCourseDeg()` (§3.2) instead of `peer->gps.groundSpeed`/`groundCourse` directly — which is strictly better twice over, since the filter's velocity state is both less noisy *and* updated every sample (not gated behind the leader's 1-in-5-packet `groundCourse`/`groundSpeed` rotation, §1.5).

### 3.4 Follower-side filter

A second, independent `FollowPositionFilter selfFilter` member, fed from a **new** `IFollowGnss` method:

```cpp
// New: a raw (unfiltered) snapshot of the follower's own position, for
// FollowManager's own filter to consume. Distinct from horizontalDistanceTo()/
// courseTo() below, which stay untouched.
virtual GNSSLocation getSelfLocation() = 0;
```

implemented in production (`FollowProdAdapters.cpp`) as `return GNSSManager::getSingleton()->getLocation();` — a thin passthrough, no `GNSSManager` changes needed.

**Why the follower-side filter lives in `FollowManager`, not inside `GNSSManager::horizontalDistanceTo()`/`courseTo()`:** those two methods (`GNSSManager.cpp:161-171`) are shared infrastructure — `PeerManager` also calls them, for every peer's `distance`/`direction` fields (`PeerManager.cpp:180-181`), used by the peer table and OLED display (`Display.cpp:281-288`). Filtering *inside* those shared methods would silently add lag to every peer's live distance/bearing readout, not just the one `FollowManager` cares about — a blast-radius change well beyond what was asked for. Keeping the filter local to `FollowManager`, and having `FollowManager` do its own distance/bearing math against the filtered self-location, avoids that entirely.

That math reuses the two new `GNSSManager` static helpers the `PATH` spec already introduces (`distanceBetween()`/`bearingBetween()`, that spec's §3.2) instead of `gnss->horizontalDistanceTo()`/`courseTo()`. All three existing call sites of the latter inside `FollowManager.cpp` switch over:

- `resolveHeadingDeg()`'s `FOLLOW_HEADING_POINT_LEADER` branch (`FollowManager.cpp:212-222`): `gnss->courseTo(leaderLoc)` → `GNSSManager::bearingBetween(filteredLocation(selfFilter), leaderLoc)` (and `leaderLoc` itself could reasonably become `filteredLocation(leaderFilter)` too, for a leader-position origin consistent with everything else this spec changes).
- `horizontalOffsetM()` (`FollowManager.cpp:270-276`, shared by `resolveAlongTrackErrorM()` and `updateDebugGvars()`): both its `gnss->horizontalDistanceTo(loc)` and `gnss->courseTo(loc)` calls switch to the `distanceBetween`/`bearingBetween` statics against `filteredLocation(selfFilter)`.
- `targetTooFar()` (`FollowManager.cpp:495-499`): same substitution.

Reset: unlike the leader filter, there's no "lock" event to key off — the follower's own position is continuous regardless of peer-lock state or the follow switch. This iteration resets `selfFilter` only on first use (lazy-initialized the same way `FollowFilterAxis1D::initialized` already handles a fresh filter) and does **not** attempt to detect and reset on GNSS fix-loss/reacquire — flagged as an open question (§7) rather than solved speculatively.

---

## 4. Capture/Emit Decoupling in `loop()`

### 4.1 The problem

`FollowManager::loop()`'s very first substantive check is a throttle:

```cpp
void FollowManager::loop()
{
    if (sys.phase <= MODE_OTA_SYNC) { return; }
    if (millis() < nextRunTime) { return; }   // <-- everything below is skipped
    nextRunTime = millis() + (1000 / config.emitHz);
    ...
    const peer_t *peer = resolveLock();   // FollowManager.cpp:557 -- first read of peer->gps
```

`peer->gps` is a single-slot struct (`PeerManager.h`) that `RadioManager::receive()` (`RadioManager.cpp:119-141`) overwrites in place whenever a new packet arrives, independent of `FollowManager`'s own call rate. Main loop calls `FollowManager::loop()` every iteration — not throttled itself, only this function's *body* is — so if two leader updates land within one `emitHz` window (250ms at the default 4Hz), the earlier one is silently clobbered before either the leader filter or (once implemented) `PATH` mode's breadcrumb buffer ever sees it. This isn't just under-sampling; it's a structural sample-loss bug once anything downstream wants finer-grained history than "whatever's newest at the throttled instant."

### 4.2 The fix

Split `loop()` into an unconditional capture step and a throttled compute+emit step. Capture stays gated on `followSwitchActive()` (preserving today's "no lock acquisition without the follow switch" semantics — see the discussion this spec is based on for why hoisting `resolveLock()` above *that* check as well was rejected), but not on `nextRunTime`:

```cpp
void FollowManager::loop()
{
    if (sys.phase <= MODE_OTA_SYNC) { return; }

    if (!followSwitchActive())
    {
        state = FOLLOW_LOCK_IDLE;
        lockedId = 0;
        lockedName[0] = '\0';
        haveValidCourse = false;
        // TODO(PATH mode): resetPathBuffer() also belongs here once implemented.
        resetLeaderFilter();
    }
    else
    {
        const peer_t *peer = resolveLock();          // runs every call now, not throttled
        if (peer != nullptr)
        {
            updateLeaderFilter(peer);                 // SS3.2, dedup'd on peer->updated
        }
    }
    updateSelfFilter();                                // SS3.4, independent of switch/lock state

    if (millis() < nextRunTime) { return; }
    nextRunTime = millis() + (1000 / config.emitHz);

    // ... unchanged from here: RC pre-arm check, re-fetch the locked peer,
    // offset/target/altitude computation, waypoint send. bail()'s
    // FOLLOW_CONDITION_NONE path on a switch-inactive/no-peer cycle is
    // unchanged in substance, just now reached after capture instead of before it.
}
```

Note the RC pre-arm check block (`FollowManager.cpp:521-545`) was already unconditional (it runs before the old `followSwitchActive()` check, independent of switch state, by design) — it stays exactly where it is, just now after the capture step instead of immediately after the `nextRunTime` throttle.

### 4.3 Why `resolveLock()` is safe to hoist

`resolveLock()` (`FollowManager.cpp:94-168`) is cheap and self-contained: a peer-table lookup (`getPeerById()` or a linear scan over `NODES_MAX` = 6), a staleness timestamp compare, a couple of string compares — no radio I/O, no trig. Its only side effects are on its own state (`state`, `lockedId`, `lockedName`), so calling it every `loop()` invocation instead of only at `emitHz` costs nothing meaningful (consistent with §6's broader conclusion that none of this is CPU-constrained). The one real decision it surfaces is the switch-gating question already resolved above: acquisition still only starts once the pilot engages follow, it just no longer waits up to `1/emitHz` to notice a switch flip in either direction.

---

## 5. Configuration Model

Two new `FollowRuntimeConfig` fields (`FollowConfig.h` defaults, `FollowManager.h` struct):

| Field | Type | Default | Meaning |
|---|---|---|---|
| `positionFilterEnabled` | `bool` | `true` | Master on/off switch, web-UI-editable. |
| `positionFilterStrengthPct` | `uint8_t` | `50` | 0-100, applied identically to the leader and follower filter instances (no platform branching, §1.4). |

**Why one knob, not raw `alpha`/`beta`:** exposing two independent fractional gains (0.0-1.0) to a pilot is both harder to reason about and mechanically awkward — `html/components.js`'s `TextValue` (used by every numeric `Setting`) calls `parseInt()` before a value ever reaches state (`FollowManager.h:132-146`'s comment on why every other geometry/timing field is stored as `int16_t`), so a raw fractional gain typed into the existing UI pattern would silently truncate to 0. Instead, `positionFilterStrengthPct` is a plain 0-100 integer (fits the existing pattern exactly, no UI changes needed beyond a new field), and `alpha`/`beta` are derived from it internally via a fixed critically-damped coupling (`alpha = positionFilterStrengthPct / 100.0`, `beta` from the standard alpha-beta critically-damped relation — exact constant to be pinned down during implementation/bench-tuning, §7). `positionFilterEnabled = false` bypasses the filter entirely — `filteredLocation()`/`filteredCourseDeg()`/`filteredSpeedMps()` degrade to their raw inputs, so disabling reproduces today's exact behavior (a required regression check, §8, since this ships enabled by default).

**Validation (`applyConfig()`):** `positionFilterStrengthPct` rejected outside `[0, 100]` (same pattern as every other bounded field, e.g. `statusGvarIndex`'s `-1`-or-`0-7` check).

**EEPROM:** two new `FollowEepromRecord` fields, `bool positionFilterEnabled` (`DIRECT`) and `int16_t positionFilterStrengthPct` (already integer, no narrowing conversion needed — same treatment as `targetPeer`/`emitHz`). Bumps `FOLLOW_EEPROM_VERSION` by 1 from whatever it is at implementation time (currently `6`; the `PATH` mode spec's own draft also proposes a bump to `7` — whichever of the two specs is implemented first takes `7`, the other takes `8`; coordinate at implementation time rather than hardcoding a number here).

---

## 6. RAM & CPU Budget

**RAM:** two `FollowPositionFilter` instances (leader + follower), each three `FollowFilterAxis1D` channels (2 doubles + 1 `unsigned long` + 1 `bool` ≈ 24 bytes/channel) plus an `origin`/`haveOrigin` pair — roughly 100-120 bytes per instance, ~220-250 bytes total. Fixed-size, doesn't scale with anything (§2.1) — smaller than the `PATH` spec's own already-accepted 896-byte breadcrumb buffer (that spec's §3.1 explicitly calls that "negligible even on the ESP8266 targets this repo ships"), and this is on top of it, not multiplied by it.

**CPU:** each filter update is a handful of multiply-adds per axis — no trig, no `sqrt` (`updateAxis1D()`, §3.1). `FollowManager` already runs full great-circle math (`sin`/`cos`/`atan2` in `GNSSManager.cpp:145-159`, plus `calculatePointAtDistance()`) every `emitHz` cycle in `slotToLatLon()`, measured live via `STATS_KEY_FOLLOWMANAGER_LOOPTIME_US` — the filter is cheaper than work already happening in this same function, and (per §4) runs once per new sample, not per `loop()` tick when nothing new has arrived (the `dtS <= 0.0` guard in `updateAxis1D()` and the peer-side dedup both make a no-op update trivially cheap).

---

## 7. Open Questions

- **Exact `alpha`/`beta` coupling and the default `positionFilterStrengthPct` value.** `50` is a placeholder, not a bench-validated number — needs tuning against real telemetry rates (parent spec's LoRa cycle time, §8 there; ESP-NOW's own rate) the same way `speedCorrectionAccelCmS2` shipped at a placeholder (`0`, "until bench-tuned") rather than a guessed default.
- **Tangent-plane origin re-anchoring.** §3.1's flat-earth projection anchors once per filter-reset and never re-centers. Negligible error at typical follow distances (tens of meters), unvalidated for a very long flight or a large-radius loiter where accumulated projection error might matter. Whether/when to re-anchor (and how to do so without a discontinuity in the filter's own state) is deferred rather than speculatively designed.
- **Follower-side filter reset on GNSS fix-loss/reacquire.** §3.4 deliberately doesn't attempt this — needs the right hook into `GNSSManager`'s fix-state transitions to detect cleanly, and it's not yet clear whether a fix recovery produces a jump large enough that the filter's own prediction can't just absorb it without an explicit reset.
- **Single conservative gain vs. platform-aware tuning (§1.4).** Deliberately deferred this iteration in favor of simplicity; revisit if bench/flight testing surfaces a specific airframe class (e.g. an aggressive multirotor) where the single gain measurably underperforms.
- **Interaction with a peer-lock ID-reuse event** (parent spec §6.3's caveat, also flagged in the `PATH` spec's own §10) — confirm during implementation that the leader filter's reset (§3.2) actually fires on that specific path, since it's a less obvious trigger than the ordinary gate-off/`forceReacquire()` cases.

---

## 8. Files / Modules to Change

1. **`src/lib/Follow/FollowConfig.h`** — `FOLLOW_POSITION_FILTER_ENABLED` (default `true`), `FOLLOW_POSITION_FILTER_STRENGTH_PCT` (default `50`) compile-time defaults.
2. **`src/lib/Follow/FollowManager.h`** — `FollowFilterAxis1D`/`FollowPositionFilter` structs; `leaderFilter`/`selfFilter` members; `positionFilterEnabled`/`positionFilterStrengthPct` on `FollowRuntimeConfig`/`FollowEepromRecord`; new private methods `updateLeaderFilter()`, `updateSelfFilter()`, `resetLeaderFilter()`, `filteredLocation()`, `filteredCourseDeg()`, `filteredSpeedMps()`, `resolveFilterGains()`.
3. **`src/lib/Follow/FollowManager.cpp`** — `loop()` restructured per §4; `resolveLock()`'s reset call sites gain `resetLeaderFilter()` (§3.2); `resolveCourseDeg()`, `slotToLatLon()`'s call site, the altitude sum, `resolveHeadingDeg()`, `horizontalOffsetM()`, and `targetTooFar()` all switched to filtered inputs (§3.2, §3.4); `applyConfig()` gains the new fields' validation; `configJson()`/`toEepromRecord()`/`fromEepromRecord()`/`statusJson()` gain the two new fields (plus, for `statusJson()`, whether each filter is initialized — mirrors the existing `haveLastTarget`-gated pattern).
4. **`src/lib/Follow/FollowDeps.h`** — `IFollowGnss` gains `getSelfLocation()`.
5. **`src/lib/Follow/FollowProdAdapters.h`/`.cpp`** — implements `getSelfLocation()` as a thin passthrough to `GNSSManager::getSingleton()->getLocation()`.
6. **`html/follow.js`** — a "Position Filtering" enable toggle + strength field (0-100), in the geometry panel or a new small panel alongside it; status panel shows whether each filter is initialized.
7. **`test/test_follow_native/`** — new `test_position_filter.cpp`: feed `updateAxis1D()` synthetic noisy input around a known smooth signal and assert the filtered output tracks it with materially less variance; assert `positionFilterEnabled = false` reproduces the pre-filter raw values exactly (the regression check this spec's default-on rollout needs); assert the leader filter resets at all three reset points (§3.2); assert `resolveCourseDeg()`'s `minCourseSpeed` fallback still holds correctly when fed filtered speed/course. Extend (not duplicate) `PATH` mode's own eventual `test_path_tracking.cpp` to assert the breadcrumb buffer records filtered samples once both land.
8. **Config/target `.ini` files** — add `FOLLOW_POSITION_FILTER_ENABLED`/`FOLLOW_POSITION_FILTER_STRENGTH_PCT` as `build_flags` defaults, matching every other config key's pattern.
9. **`docs/spec/2026-09-08-FollowPathTracking.md`** — cross-reference note only (already added to this doc's "Depends on" header): that spec's §3.1 buffer-push and its own `loop()` integration sketch (§3.3 there) should be reconciled with this spec's §3.2/§4 at implementation time rather than implemented independently.

---

## 9. Test & Acceptance

### 9.1 Bench (props off)

1. **Noise rejection, the core acceptance test:** spoof a peer on a straight or hex-path track (`spoofPeerHexPath()`) with injected synthetic position jitter; compare `LIVE` mode's `lastTarget` series with `positionFilterEnabled = true` vs. `false` and confirm materially less cycle-to-cycle variance with filtering on, without introducing perceptible lag through a turn at the default strength.
2. **Disable = exact regression.** Confirm `positionFilterEnabled = false` reproduces the pre-filter target/course/heading output bit-for-bit (floating-point rounding aside) against the same spoofed input — required since this feature ships **enabled by default**, changing existing installs' live behavior the moment it's flashed.
3. **Capture runs unthrottled.** Confirm the leader/follower filters' `lastUpdateMs` advance faster than `1/emitHz` would allow when the spoofed peer's telemetry updates faster than `emitHz`, while waypoint emission itself still only happens at `emitHz` (§4).
4. **Filter reset on re-lock.** Lock peer A, let its filter converge, force-reacquire onto peer B, confirm the leader filter's next estimate is peer B's raw first sample, not a blend with peer A's trail (§3.2).
5. **RAM check.** `ESP.getFreeHeap()` before/after enabling, confirm the delta matches §6's ~250-byte estimate.
6. **`minCourseSpeed` fallback still works.** Drive the spoofed peer's speed below `minCourseSpeed` and confirm course holds at `lastValidCourseDeg` (now filter-derived) exactly as it does today (§3.3).

### 9.2 Flight (progressive, open area, big margins)

1. A/B the default `LIVE` mode with filtering enabled vs. disabled on the same gentle-turn course; confirm visibly smoother tracking with filtering on, no added perceptible lag, and instant manual override still works.
2. Progress to tighter maneuvering only after step 1 is confirmed safe.

### 9.3 Acceptance criteria

- With `positionFilterEnabled = true` (the default), both `LIVE` mode's target and the follower's own self-position-derived readings (heading, along-track error, target-distance check) show materially reduced cycle-to-cycle jitter versus today's raw inputs, on both bench (9.1.1) and flight (9.2).
- `positionFilterEnabled = false` reproduces today's exact behavior (9.1.2) — the required safety net for a default-on change.
- Leader/follower filter capture (§4) runs every `loop()` call, decoupled from `emitHz`, without increasing measured `FOLLOWMANAGER_LOOPTIME_US` beyond a negligible margin (§6).
- RAM usage increase stays within the estimate in §6, confirmed empirically (9.1.5).
- All existing follow behavior unrelated to this feature (offset geometry, altitude floor, RC scaling, autothrottle, peer-lock semantics) is unchanged.
