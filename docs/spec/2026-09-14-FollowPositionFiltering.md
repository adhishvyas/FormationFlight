# FormationFlight — Follow Position/Velocity Filtering — Engineering Spec

**Status:** Implemented, pending bench/flight validation (§9.1.5's RAM check was confirmed by
measuring the compiled struct sizes rather than a live on-device `free_heap` read — see
[the plan](../plans/2026-10-01-FollowPositionFiltering-Plan.md)'s phase G notes; §9.2's flight
test still needs real hardware and is unchanged as the final manual sign-off gate)

> *Note (2026-10-01):* this spec was originally drafted referencing v1 file paths
> and seams (`src/lib/Follow/FollowManager.cpp`, `GNSSManager`, `PeerManager`,
> `IFollowGnss`, EEPROM persistence). Those don't build into any target anymore
> — Follow now runs as `ff::FollowController` in `lib/ff_core/follow.h/.cpp`,
> peers are 32-bit UIDs in `ff::PeerTable`, config is one JSON `Settings`
> document (`lib/ff_core/config.h`, `hal/ConfigStore`), and the follower's own
> position comes from `ff::ILocationSource` directly — there is no shared
> `GNSSManager`-style seam to route around. This revision corrects every file
> reference and seam below to the real v2 code; the design intent (recursive
> alpha-beta filtering, upstream of track-mode consumption) is unchanged from
> the original draft. See
> [`../plans/2026-10-01-FollowPositionFiltering-Plan.md`](../plans/2026-10-01-FollowPositionFiltering-Plan.md)
> for the phased implementation plan and the authoritative file-by-file
> breakdown.

**Target firmware:** FormationFlight (ESP32/ESP8266, PlatformIO/C++)
**Depends on:**
- [`2026-07-31-FollowMeOnInav.md`](2026-07-31-FollowMeOnInav.md) — the parent spec. This addendum reuses its offset/geometry/altitude-floor model, now `ff::FollowConfig`/`ff::FollowController` (`lib/ff_core/follow.h/.cpp`) rather than the v1 `FollowRuntimeConfig`/EEPROM design the original draft cited.
- [`2026-09-08-FollowPathTracking.md`](2026-09-08-FollowPathTracking.md) — **not yet implemented** (no `PATH` mode, no breadcrumb buffer exists in the tree as of this writing). The original draft assumed this spec would land first and introduce a shared `GNSSManager::distanceBetween()`/`bearingBetween()` pair this spec would reuse; v2 already has equivalent free functions (`ff::geo::distanceM()`/`ff::geo::bearingDeg()`, `lib/ff_core/geo.h`), so no new statics are needed regardless of landing order. If/when `PATH` mode is implemented, its breadcrumb buffer should record the *filtered* leader samples this spec produces, not raw ones — that integration is deferred to whichever spec/plan lands second, not designed here.
- [`2026-08-28-FollowSpeedAutothrottle.md`](2026-08-28-FollowSpeedAutothrottle.md) — already implemented (`FollowConfig`'s `speedCorrectionAccelCmS2`/`minTargetSpeedMps`/`maxTargetSpeedMps`/autothrottle GVAR fields). The follower-side filter (§3.4) feeds cleaner inputs into that feature's along-track error computation; no change to the autothrottle's own logic.

---

## 1. Purpose & Scope

### 1.1 Problem

GPS position fixes are noisy — ordinary receiver scatter, not gross error — for both the leader (received over the radio link) and the follower (its own onboard/FC-attached GNSS). Today `FollowController` consumes both raw, every cycle: `LIVE` mode's `slotToLatLon(peer->lat, peer->lon, courseDeg, ...)` call in `FollowController::service()` (`lib/ff_core/follow.cpp`) projects directly from the leader's current raw lat/lon rotated by its raw `course_ddeg`-derived `courseDeg` (`FollowController::resolveCourseDeg()`), and the follower's own position is read fresh, unfiltered, every time `ILocationSource::getLocation()` is called (`self_->getLocation()`, called directly inside `resolveLock()` and `service()` — there is no shared distance/bearing helper in between to route around). The visible result is a commanded target that jitters cycle to cycle even when the leader is flying a physically smooth line, and — since `LIVE` mode's rotation angle comes from the same noisy `course_ddeg` field — a target that can also swing directionally on course jitter alone, independent of any real leader maneuver.

### 1.2 Goal

Smooth both the leader's and the follower's own position/velocity with one lightweight recursive filter design, applied **upstream of whichever track mode consumes it** — so `LIVE` mode (today's only mode) and `PATH` mode (the addendum spec, not yet implemented) both benefit from the same noise rejection, rather than filtering being bolted on as a `PATH`-only feature.

### 1.3 In scope

- One position/velocity filter instance tracking the locked leader, fed from received peer telemetry (§3.1-§3.2).
- One position/velocity filter instance tracking the follower's own position, fed from `ILocationSource::getLocation()` — the same call `FollowController` already makes directly, no new interface method needed (§3.4).
- A capture/emit decoupling of `FollowController::service()` (§4) so both filters update every `service()` call — bounded only by however often new telemetry/GNSS samples actually arrive — instead of being throttled to `config.emitHz` alongside the rest of the follow computation.
- Two new `FollowConfig` fields: one enable flag, one single tuning knob, applied identically to both filter instances (§5) — deliberately not platform-aware (multirotor vs. fixed-wing) in this iteration; see §1.4 and §7.
- `LIVE` mode's `slotToLatLon()` call site, `resolveCourseDeg()`, `resolveHeadingDeg()`'s `FOLLOW_HEADING_POINT_LEADER` branch, `resolveAlongTrackErrorM()`/`updateDebugGvars()`'s shared `horizontalOffsetM()` call sites, and `targetTooFar()` all switched to read the relevant filtered estimate instead of a raw sample (§3.2, §3.4).

### 1.4 Out of scope (this iteration)

- **Any leader-side or protocol change.** The filter runs entirely off data the follower already receives today — same non-goal framing as both specs this one depends on.
- **Per-platform (multirotor vs. fixed-wing) filter tuning.** Considered (see conversation leading to this spec) and deliberately deferred: a single conservative gain, with the existing `minCourseSpeed` fallback still underneath it as a hover/near-zero-speed safety net (§3.3), is simpler and was judged adequate for this iteration. The follower's own mixer platform type is in fact already known for free (`IFollowFc::platformType()`, used today to gate the fixed-wing-only speed autothrottle) — a follow-up could branch the follower-side gain on it cheaply — but this iteration uses one gain for both the leader-side and follower-side filter, no platform branching anywhere.
- **A pilot-supplied "expected leader platform" hint config field.** Explicitly rejected for this iteration in favor of the single-gain approach above — the leader's platform type isn't in the radio packet (`PositionPacket`, `lib/ff_core/protocol.h`) and adding a config field plus a leader-side behavior branch wasn't judged worth it for the marginal tuning improvement. May be revisited if bench/flight testing shows the single gain performs poorly for one airframe class.
- **Replacing `minCourseSpeed`'s hold-last-known-course fallback.** Kept exactly as-is, sitting underneath the filter (§3.3) — this spec changes what feeds it, not its own logic.
- **A windowed/moving-average filter, or any raw-sample history buffer for this feature specifically.** The filter design here is recursive: fixed-size state, no history array (§3.1, §6) — distinct from `PATH` mode's (not yet implemented) breadcrumb ring buffer, which is a separate feature this filter's output would feed into once it exists.
- **Filtering used anywhere outside `FollowController`.** The web status handler (`src/hal/WebServer.cpp`) computes each peer's `distance_m`/`bearing_deg`/`rel_alt_m` display fields independently, from the same raw `Peer` table entries, for the peer list and (future) OLED display. This spec does **not** touch that computation — see §3.4 for why, and the blast-radius reasoning behind keeping both filter instances local to `FollowController`.

### 1.5 Assumptions

- Leader telemetry arrives via the v2 wire protocol's position beacon (`ff::PositionPacket`, `lib/ff_core/protocol.h`): `lat`/`lon`/`alt_m`/`speed_cms`/`course_ddeg` are **all present on every position packet** — unlike v1's `air_type0_t`, there is no rotating `extra_value` slot and no 1-in-5-packet staleness for course/speed in this protocol. So the specific "stale course" argument the original draft of this spec made does not apply to v2. The filter is still justified on noise grounds alone: §2.2's argument — that differencing two already-noisy position fixes to get a velocity reading, or trusting a reported ground-course/-speed field as a filter *input* rather than letting the filter derive velocity as *state*, reinjects amplified position noise — holds regardless of how fresh that field is. Deriving the leader's velocity internally from consecutive filtered position samples (§3.2) is kept for that reason, not a staleness one.
- The follower's own `NodeLocation.speed_cms`/`course_ddeg` (`ILocationSource::getLocation()`) update at whatever rate the connected position source natively runs at (MSP-attached FC via `hal/MspFcLink`, or a directly-wired GPS via `hal/DirectGpsLocationSource`/`ubx.cpp`) — still GPS-derived and still unreliable near zero groundspeed, so it's filtered the same way for consistency (§3.4).

---

## 2. Background

### 2.1 Why a recursive filter, not a windowed one

A windowed filter (moving average, median-of-N) needs to store N raw samples — RAM scales with window size, and a wider window (more noise rejection) directly costs more lag. A recursive filter — alpha-beta, the constant-velocity-model family used here — carries only its *current* position+velocity estimate forward; each new measurement updates that estimate and is then discarded. No history array, fixed-size state regardless of how far back "smooth" needs to reach. This also happens to be the answer to the earlier RAM question: state is a handful of doubles per axis, independent of buffer capacity or flight duration (§6).

### 2.2 Why velocity is filter *state*, not a filter *input*

The naive approach — feed the leader's reported `course_ddeg`/`speed_cms` into the filter as a velocity measurement — has a fundamental problem: if that field were ever derived by differencing two noisy position fixes rather than a genuine Doppler measurement, using it as a filter input would just reinject amplified position noise (differentiation is a high-pass operation). The alpha-beta design sidesteps this entirely: velocity is *estimated internally*, from the residual between predicted and measured position at each update (§3.1's `update()`). The filter never needs to trust an external speed/course reading at all — it only ever measures position. (v2's protocol updates `course_ddeg`/`speed_cms` every packet rather than staling them as v1 did — see §1.5 — but that only removes one of two reasons to prefer this design, not both.)

### 2.3 Why this isn't `PATH`-mode-specific

`PATH` mode (the addendum spec, not yet implemented) solves corner-cutting — a **geometry/lag** problem: `LIVE` mode always projects from the leader's *current* position+course, so the target snaps the instant the leader turns, even with perfectly noise-free GPS. Filtering solves **noise** — jitter in the position/course readings themselves — which exists regardless of which track mode is active, because both `LIVE`'s live point and `PATH`'s (future) walked-back point are ultimately built from the same raw `Peer` stream. Concretely: `LIVE` mode's `slotToLatLon()` call in `FollowController::service()` reads raw `peer->lat`/`lon` and a raw-`course_ddeg`-derived `courseDeg` **today**, with no dependency on `PATH` mode existing at all — so filtering is a strict improvement to the mode already shipped, not just infrastructure for a future one.

---

## 3. Filter Design

### 3.1 State & algorithm

One filter instance covers one tracked point (the leader, or the follower itself) as three independent 1D alpha-beta channels — north, east (a local flat-earth tangent-plane projection, not raw lat/lon degrees — see below for why), and altitude:

```cpp
// One recursive position+velocity channel. No history -- see SS2.1.
struct FollowFilterAxis1D {
    double position = 0.0;       // filtered estimate (meters, local frame)
    double velocity = 0.0;       // filtered estimate (m/s)
    uint32_t lastUpdateMs = 0;
    bool initialized = false;
};

struct FollowPositionFilter {
    double originLat = 0.0;       // local tangent-plane anchor -- set once, SS3.1
    double originLon = 0.0;
    bool haveOrigin = false;
    FollowFilterAxis1D north;     // meters north of origin
    FollowFilterAxis1D east;      // meters east of origin
    FollowFilterAxis1D alt;       // meters MSL -- see SS3.2/SS3.4 for which raw field feeds each instance
};
```

Per-channel update (the same function, called three times per filter update — once for north, east, and altitude):

```cpp
// alpha, beta in (0, 1], derived from config.positionFilterStrengthPct (SS5).
void updateAxis1D(FollowFilterAxis1D *axis, double measurement, uint32_t nowMs, double alpha, double beta)
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

**Why a local north/east tangent plane, not raw lat/lon degrees:** a longitude degree's length in meters shrinks by `cos(latitude)` — filtering raw lat/lon independently would give a `velocity` state with no consistent meaning in meters/second, making a derived course (`atan2`) meaningless. Instead, `FollowPositionFilter`'s origin is set once (the *first* sample this filter instance sees after a reset, per §3.3/§3.4's reset points) via a flat-earth equirectangular projection: `northM = (lat - originLat) * metersPerDegLat`, `eastM = (lon - originLon) * metersPerDegLon(originLat)` (`metersPerDegLon` scaled by `cos(originLat)`, computed once when the origin is set). Error from the flat-earth approximation is negligible at the distances involved here (tens of meters of GPS scatter, `maxTargetDistM` on the order of tens of meters) — see §7 for the long-flight/large-loiter caveat this simplification defers.

Reading the filter back out (used by both `LIVE` mode and, later, `PATH` mode's breadcrumb push):

```cpp
NodeLocationLike filteredLocation(const FollowPositionFilter &f);  // re-projects north/east through origin back to lat/lon (deg*1e7), alt in metres
double filteredCourseDeg(const FollowPositionFilter &f);           // atan2(east.velocity, north.velocity), wrapped [0,360)
double filteredSpeedMps(const FollowPositionFilter &f);            // hypot(north.velocity, east.velocity)
```

(`filteredLocation()`'s exact return shape — a small value type, not a reused `NodeLocation`/`Peer` struct, since neither carries exactly the filtered fields needed — is an implementation-time choice made in the plan, not fixed here.)

### 3.2 Leader-side filter

One `FollowPositionFilter leaderFilter_` member on `FollowController`, updated whenever a new leader sample arrives — dedup'd on the peer's `last_position_ms` (so a cycle where the peer table hasn't been refreshed doesn't feed a duplicate timestamp into `updateAxis1D()`, which already no-ops on `dtS <= 0`) — feeding `peer->lat`/`lon` (deg*1e7, converted the same way `slotToLatLon()` already does) and `peer->alt_m` (metres MSL) into the filter's three channels.

Reset (`resetLeaderFilter()`, reinitializing `leaderFilter_` to its default-constructed state) at exactly the points a future `PATH` mode's breadcrumb buffer would also need resetting — a freshly (re-)locked peer's filter state must start fresh, or the filter's first few estimates after a peer swap would be a weighted blend of the old and new leader's positions:
- `FollowController::resolveLock()`'s transition into `FOLLOW_LOCK_ACQUIRING` state assigning a new `lockedUid_` (the `ACQUIRING → LOCKED` lock event).
- `FollowController::forceReacquire()`.
- `FollowController::service()`'s gate-inactive branch (where `state_`/`lockedUid_`/`lockedName_`/`haveValidCourse_` are already reset today).

Note: v2's peer identity is a stable 32-bit UID (`Peer::uid`), not a v1 slot number that could be silently reused by a different aircraft — so there is no separate "lock ID reuse" edge case to worry about here beyond the three reset points above.

**Consumers**, both switched from raw to filtered:
- `LIVE` mode's `slotToLatLon()` call in `service()` — reads `filteredLocation(leaderFilter_)` instead of `peer->lat`/`lon`.
- `resolveCourseDeg()` — its `minCourseSpeed` comparison and `lastValidCourseDeg_` both switch from `peer->speed_cms`/`course_ddeg` to `filteredSpeedMps(leaderFilter_)`/`filteredCourseDeg(leaderFilter_)` (§3.3).
- The altitude sum in `service()` (today `peer->alt_m - self.alt_m`) becomes `leaderFilter_.alt.position - selfFilter_.alt.position` — both terms filtered, not just the leader's (§3.4 covers why the follower side needs its own instance rather than reusing raw `self.alt_m` here).

### 3.3 Interaction with `minCourseSpeed`

`resolveCourseDeg()`'s existing hold-last-known-course fallback (below `minCourseSpeed`, hold `lastValidCourseDeg_` rather than following GPS course jitter while stationary) is kept exactly as-is — this is the hover/near-zero-groundspeed safety net that matters far more for multirotor leaders (which can genuinely hover) than fixed-wing ones (§1.4). Only its *inputs* change: the comparison and the held/reported value both come from the leader filter's own `filteredSpeedMps()`/`filteredCourseDeg()` (§3.2) instead of `peer->speed_cms`/`course_ddeg` directly — which is strictly better, since the filter's velocity state is less noisy than the raw field (the staleness advantage the original draft also claimed does not apply in v2 — see §1.5).

### 3.4 Follower-side filter

A second, independent `FollowPositionFilter selfFilter_` member, fed from the same call `FollowController` already makes: `self_->getLocation()` (`ILocationSource::getLocation()`). **No new interface method is needed** — unlike the v1 draft of this spec, which proposed a new `IFollowGnss::getSelfLocation()` seam specifically to avoid adding filtering lag to a *shared* `GNSSManager::horizontalDistanceTo()`/`courseTo()` pair that v1's `PeerManager` also called for every peer's OLED/peer-table distance display. That shared pair doesn't exist in v2: `FollowController` already owns a private `ILocationSource* self_` and calls `getLocation()` directly wherever it needs the follower's own fix, and the web status handler's peer `distance_m`/`bearing_deg` fields (`src/hal/WebServer.cpp`) are computed independently, from the raw peer table, with no code path through `FollowController` at all. So the blast-radius concern the original draft was solving for doesn't arise here — there is nothing to route around.

That math reuses `ff::geo::distanceM()`/`ff::geo::bearingDeg()` (`lib/ff_core/geo.h`, already used throughout `follow.cpp` today) in place of the raw-`self` arguments they currently take. The call sites inside `follow.cpp` that switch over:

- `resolveHeadingDeg()`'s `FOLLOW_HEADING_POINT_LEADER` branch: `geo::bearingDeg(self.lat, self.lon, peer.lat, peer.lon)` → bearing from `filteredLocation(selfFilter_)` to `filteredLocation(leaderFilter_)` (both ends filtered, for a self-position-and-leader-position origin consistent with everything else this spec changes).
- `horizontalOffsetM()` (the free helper shared by `resolveAlongTrackErrorM()` and `updateDebugGvars()`): its `self` argument switches from the raw `NodeLocation` to `filteredLocation(selfFilter_)`.
- `targetTooFar()`: same substitution — `self` becomes `filteredLocation(selfFilter_)`.

Reset: unlike the leader filter, there's no "lock" event to key off — the follower's own position is continuous regardless of peer-lock state or the follow switch. This iteration resets `selfFilter_` only on first use (lazy-initialized the same way `FollowFilterAxis1D::initialized` already handles a fresh filter) and does **not** attempt to detect and reset on GNSS fix-loss/reacquire — flagged as an open question (§7) rather than solved speculatively.

---

## 4. Capture/Emit Decoupling in `service()`

### 4.1 The problem

`FollowController::service()`'s very first substantive check is a throttle:

```cpp
void FollowController::service(uint32_t now_ms) {
    if (started_ && static_cast<int32_t>(now_ms - nextRunMs_) < 0) {
        return;   // <-- everything below is skipped
    }
    started_ = true;
    nextRunMs_ = now_ms + (1000u / config_.emitHz);
    ...
    const Peer* peer = resolveLock(now_ms);   // first read of the locked peer's fields
```

`Peer` entries are overwritten in place by `PeerTable::updatePosition()` whenever a new packet arrives, independent of `FollowController::service()`'s own call rate. `main.cpp`'s `loop()` calls `g_follow->service(now)` every iteration — not throttled itself, only this function's *body* is — so if two leader updates land within one `emitHz` window (250ms at the default 4Hz), the earlier one is silently clobbered before either the leader filter or (once implemented) `PATH` mode's breadcrumb buffer ever sees it. This isn't just under-sampling; it's a structural sample-loss bug once anything downstream wants finer-grained history than "whatever's newest at the throttled instant."

### 4.2 The fix

Split `service()` into an unconditional capture step and a throttled compute+emit step. Capture stays gated on `followSwitchActive()` (preserving today's "no lock acquisition without the follow switch" semantics — see the discussion this spec is based on for why hoisting `resolveLock()` above *that* check as well was rejected), but not on `nextRunMs_`:

```cpp
void FollowController::service(uint32_t now_ms) {
    const bool gateActive = followSwitchActive();
    fc_->setTelemetryNeeds(gateActive,
                           anyRcChannelAssigned() || config_.autothrottleEnableRcChannel >= 1);

    if (!gateActive) {
        state_ = FOLLOW_LOCK_IDLE;
        lockedUid_ = 0;
        lockedName_[0] = '\0';
        haveValidCourse_ = false;
        resetLeaderFilter();                  // SS3.2
    } else {
        const Peer* peer = resolveLock(now_ms);   // runs every call now, not throttled
        if (peer != nullptr && config_.positionFilterEnabled) {
            updateLeaderFilter(peer, now_ms);      // SS3.2, dedup'd on last_position_ms
        }
    }
    if (config_.positionFilterEnabled) {
        updateSelfFilter(now_ms);                  // SS3.4, independent of switch/lock state
    }

    if (started_ && static_cast<int32_t>(now_ms - nextRunMs_) < 0) {
        return;
    }
    started_ = true;
    nextRunMs_ = now_ms + (1000u / config_.emitHz);

    // ... unchanged from here: the RC pre-arm check, re-fetch the locked peer,
    // offset/target/altitude computation (reading the filtered estimates when
    // config_.positionFilterEnabled, the raw peer/self fields otherwise -- see
    // SS5 for why this is a hard branch rather than a gain-based degrade),
    // waypoint send.
}
```

The RC pre-arm check block, today unconditional (it runs regardless of switch state, by design), stays exactly where it is — after the capture step instead of immediately after the old throttle.

### 4.3 Why `resolveLock()` is safe to hoist

`resolveLock()` is cheap and self-contained: a peer-table lookup (`PeerTable::find()`, or a linear scan over `PeerTable::capacity()` for the nearest-peer case), a staleness timestamp compare (`followPeerStale()`), a couple of string compares — no radio I/O, no trig. Its only side effects are on its own state (`state_`, `lockedUid_`, `lockedName_`), so calling it every `service()` invocation instead of only at `emitHz` costs nothing meaningful (consistent with §6's broader conclusion that none of this is CPU-constrained). The one real decision it surfaces is the switch-gating question already resolved above: acquisition still only starts once the pilot engages follow, it just no longer waits up to `1/emitHz` to notice a switch flip in either direction.

---

## 5. Configuration Model

Two new `FollowConfig` fields (`lib/ff_core/follow.h`), with compile-time `#ifndef`-guarded defaults the same way every other Follow field works:

| Field | Type | Default | Meaning |
|---|---|---|---|
| `positionFilterEnabled` | `bool` | `true` | Master on/off switch, web-UI-editable. |
| `positionFilterStrengthPct` | `uint8_t` | `50` | 0-100, applied identically to the leader and follower filter instances (no platform branching, §1.4). 0 = lightest smoothing (filter nearly reproduces the raw signal), 100 = heaviest. |

**Why one knob, not raw `alpha`/`beta`:** exposing two independent fractional gains (0.0-1.0) to a pilot is both harder to reason about and mechanically awkward — `html/components.js`'s `TextValue` (used by every numeric `Setting` row, including every other geometry/timing field in `html/follow.js`) parses with `parseInt()`, so a raw fractional gain typed into the existing UI pattern would silently truncate to 0. Instead, `positionFilterStrengthPct` is a plain 0-100 integer (fits the existing pattern exactly), and `alpha`/`beta` are derived from it internally.

**Gain derivation (resolved, not left as a placeholder):** `alpha` and `beta` use the standard Benedict-Bordner critically-damped g-h filter relation — chosen specifically because it has a closed form in terms of a single parameter and is a well-known, no-overshoot default for this filter family:

```cpp
// positionFilterStrengthPct: 0 (lightest) .. 100 (heaviest).
// kMinAlpha keeps strength=100 from degenerating into a literal freeze
// (alpha=0 would mean the estimate never again moves after its first sample).
constexpr double kFollowFilterMinAlpha = 0.02;

double alpha = 1.0 - (positionFilterStrengthPct / 100.0);
if (alpha < kFollowFilterMinAlpha) { alpha = kFollowFilterMinAlpha; }
double beta = (alpha * alpha) / (2.0 - alpha);   // Benedict-Bordner critically damped relation
```

At the default `positionFilterStrengthPct = 50`: `alpha = 0.5`, `beta ≈ 0.167`. At `0`: `alpha = 1`, `beta = 1` (each update takes the raw measurement and a one-step velocity difference — minimal smoothing, closest to today's unfiltered behavior while still running through the filter machinery). At `100`: `alpha = 0.02` (floored), `beta ≈ 0.0002` — heavy smoothing, slow to react. `50` remains a placeholder in the sense that it hasn't been bench/flight validated against this project's actual telemetry rates (§7) — but the *formula* is fixed, so retuning later is a single-constant config change (`positionFilterStrengthPct`), not a code change.

**`positionFilterEnabled = false` behavior:** rather than letting `alpha`/`beta` degrade toward an identity transform (which would only be bit-exact modulo floating-point rounding), `service()` skips the filter update and filtered-read calls entirely when disabled and falls straight through to the pre-existing raw-field reads (§4.2's capture step only calls `updateLeaderFilter()`/`updateSelfFilter()` when `positionFilterEnabled`; the compute step reads `peer->lat`/`lon`/`course_ddeg`/`speed_cms`/`alt_m` and `self.lat`/`lon`/`alt_m` directly instead of through `filteredLocation()`/`filteredCourseDeg()`/`filteredSpeedMps()`). This makes the disabled case **exactly** today's code path, not merely numerically close to it — the regression check this spec's default-on rollout needs (§8, §9.1.2) is then a tautology by construction rather than something that can regress later if the gain formula changes.

**Validation (`followValidateConfig()`):** `positionFilterStrengthPct` is already `uint8_t`, so it cannot be negative or exceed 255; reject outside `[0, 100]` the same way `statusGvarIndex` is range-checked today.

**Persistence:** no EEPROM, no version bump — v2 config is one JSON `Settings` document (`lib/ff_core/config.h`) written to LittleFS by `hal/ConfigStore`. Both new fields are plain numeric/bool members added to `FOLLOW_CONFIG_DIRECT_FIELDS` (`follow.h`), which `config.cpp`'s `mergeFollow()`/`followToJson()` macros already pick up automatically — the same mechanism every other recently-added Follow field (e.g. the autothrottle fields) used, no narrowing-conversion concerns since neither new field is a `double`.

---

## 6. RAM & CPU Budget

**RAM:** two `FollowPositionFilter` instances (leader + follower), each three `FollowFilterAxis1D` channels (2 doubles + 1 `uint32_t` + 1 `bool` ≈ 24 bytes/channel) plus an origin lat/lon/`haveOrigin` triple — roughly 100-120 bytes per instance, ~220-250 bytes total. Fixed-size, doesn't scale with anything (§2.1) — smaller than `PATH` mode's own proposed 896-byte breadcrumb buffer (not yet implemented), and this is additive, not multiplied by it, once both land.

**CPU:** each filter update is a handful of multiply-adds per axis — no trig, no `sqrt` (`updateAxis1D()`, §3.1). `FollowController` already runs full great-circle math (`sin`/`cos`/`atan2` in `lib/ff_core/geo.cpp`, plus `geo::pointAtDistance()`) every `emitHz` cycle in `slotToLatLon()`/`resolveHeadingDeg()`/`horizontalOffsetM()`, visible via the node-wide `loop` status block (`ff::LoopStats`, `docs/v2-web-api.md`'s `loop` object) — the filter is cheaper than work already happening in this same function, and (per §4) runs once per new sample, not per `service()` tick when nothing new has arrived (the `dtS <= 0.0` guard in `updateAxis1D()` and the peer-side dedup both make a no-op update trivially cheap).

---

## 7. Open Questions

- **Default `positionFilterStrengthPct` value.** `50` is a placeholder, not a bench-validated number — the *gain formula* is now fixed (§5), but the single tuning knob's default still needs validating against real telemetry rates (LoRa's and ESP-NOW's own beacon cadence) the same way `speedCorrectionAccelCmS2` shipped at a placeholder (`0`, "until bench-tuned") rather than a guessed default.
- **Tangent-plane origin re-anchoring.** §3.1's flat-earth projection anchors once per filter-reset and never re-centers. Negligible error at typical follow distances (tens of meters), unvalidated for a very long flight or a large-radius loiter where accumulated projection error might matter. Whether/when to re-anchor (and how to do so without a discontinuity in the filter's own state) is deferred rather than speculatively designed.
- **Follower-side filter reset on GNSS fix-loss/reacquire.** §3.4 deliberately doesn't attempt this — needs the right hook into the location source's fix-state transitions to detect cleanly, and it's not yet clear whether a fix recovery produces a jump large enough that the filter's own prediction can't just absorb it without an explicit reset.
- **Single conservative gain vs. platform-aware tuning (§1.4).** Deliberately deferred this iteration in favor of simplicity; revisit if bench/flight testing surfaces a specific airframe class (e.g. an aggressive multirotor) where the single gain measurably underperforms.

(The original draft's open question about a peer-lock ID-reuse interaction with the leader filter's reset does not apply to v2: peers are identified by a stable 32-bit UID, not a reusable slot number, so there is no "same ID, different aircraft" case to guard against — see §3.2's note.)

---

## 8. Files / Modules to Change

See [`../plans/2026-10-01-FollowPositionFiltering-Plan.md`](../plans/2026-10-01-FollowPositionFiltering-Plan.md) for the authoritative, phased file-by-file breakdown (kept in the plan rather than duplicated here so the two can't drift). In summary, the surface area is: `lib/ff_core/follow.h`/`follow.cpp` (filter primitive, config fields, controller wiring), `lib/ff_core/config.cpp` (JSON merge/serialize — mostly automatic via the existing field-list macros), `src/hal/WebServer.cpp` (status JSON), `docs/v2-web-api.md` (wire contract), `html/follow.js`/`html/follow-logic.js` (UI panel + client-side validation mirror), `scripts/mock_server.py` (defaults/validation/status mirror), `test/fixtures/follow-config-cases.json` (shared validator fixture consumed by the C++/Python/JS mirrors), and `test/test_follow/` (new and extended native tests).

---

## 9. Test & Acceptance

### 9.1 Bench (props off)

1. **Noise rejection, the core acceptance test:** spoof a peer on a straight or `ff::SimMode::Hex` hexagon-patrol track (or, for a native host test, `FollowHarness::setPeerAt()` fed a smooth ground-truth path plus injected synthetic jitter) and compare `LIVE` mode's resolved target series with `positionFilterEnabled = true` vs. `false` and confirm materially less cycle-to-cycle variance with filtering on, without introducing perceptible lag through a turn at the default strength.
2. **Disable = exact regression.** Confirm `positionFilterEnabled = false` reproduces the pre-filter target/course/heading output bit-for-bit against the same spoofed input — guaranteed by construction per §5's disabled-path design, but still worth a direct regression test since this feature ships **enabled by default**, changing existing installs' live behavior the moment it's flashed.
3. **Capture runs unthrottled.** Confirm the leader/follower filters' internal `lastUpdateMs` advance faster than `1/emitHz` would allow when the spoofed peer's telemetry updates faster than `emitHz`, while waypoint emission itself still only happens at `emitHz` (§4).
4. **Filter reset on re-lock.** Lock peer A, let its filter converge, force-reacquire onto peer B, confirm the leader filter's next estimate is peer B's raw first sample, not a blend with peer A's trail (§3.2).
5. **RAM check.** Free-heap delta before/after enabling (e.g. via the web status `system.free_heap` field), confirm it matches §6's ~250-byte estimate.
6. **`minCourseSpeed` fallback still works.** Drive the spoofed peer's speed below `minCourseSpeed` and confirm course holds at `lastValidCourseDeg_` (now filter-derived) exactly as it does today (§3.3).

### 9.2 Flight (progressive, open area, big margins)

1. A/B the default `LIVE` mode with filtering enabled vs. disabled on the same gentle-turn course; confirm visibly smoother tracking with filtering on, no added perceptible lag, and instant manual override still works.
2. Progress to tighter maneuvering only after step 1 is confirmed safe.

### 9.3 Acceptance criteria

- With `positionFilterEnabled = true` (the default), both `LIVE` mode's target and the follower's own self-position-derived readings (heading, along-track error, target-distance check) show materially reduced cycle-to-cycle jitter versus today's raw inputs, on both bench (9.1.1) and flight (9.2).
- `positionFilterEnabled = false` reproduces today's exact behavior (9.1.2) — the required safety net for a default-on change.
- Leader/follower filter capture (§4) runs every `service()` call, decoupled from `emitHz`, without increasing measured loop timing beyond a negligible margin (§6).
- RAM usage increase stays within the estimate in §6, confirmed empirically (9.1.5).
- All existing follow behavior unrelated to this feature (offset geometry, altitude floor, RC scaling, autothrottle, peer-lock semantics) is unchanged.
