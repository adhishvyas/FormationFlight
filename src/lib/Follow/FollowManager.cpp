#include "FollowManager.h"
#include "../MSP/MSPManager.h"
#include "../GNSS/GNSSManager.h"
#include "main.h"
#include <math.h>
#include <EEPROM.h>

// EEPROM region begins immediately after cfg's own footprint (main.h) —
// see ConfigHandler.cpp's config_init(), which reserves
// sizeof(cfg) + sizeof(FollowEepromRecord) total via EEPROM.begin() so
// this region is always available once EEPROM.begin() has run.
#define FOLLOW_EEPROM_OFFSET sizeof(cfg)

// Minimum time between EEPROM commits (Phase 4B) — guards against a
// stuck/spammed "Save to EEPROM" button hammering flash with writes.
#define FOLLOW_EEPROM_COMMIT_MIN_INTERVAL_MS 2000

// Below this horizontal offset magnitude, a slot is considered "stacked"
// (overhead/underneath) for the purposes of the minimum-vertical-separation
// rule (spec §7.4).
#define FOLLOW_STACKED_HORIZONTAL_EPSILON_M 0.5

// Tolerance for §4.6's "RC candidate must reproduce the static default"
// pre-arm rule, below — absorbs RC read quantization/jitter (frac steps in
// 1/500 increments) without weakening the check into a sign-only match.
#define FOLLOW_PREARM_MATCH_EPSILON_M 0.05

// How often to resend a GVAR even if its value hasn't changed, so a
// single dropped MSP write doesn't leave the OSD showing a stale state
// indefinitely (spec §3.3). 20x less frequent than the default 4 Hz
// waypoint stream — negligible added MSP traffic (spec §8's "don't flood
// MSP" budget, referenced by §3.3).
#define FOLLOW_GVAR_HEARTBEAT_MS 5000

FollowTarget slotToLatLon(int32_t peer_lat_1e6, int32_t peer_lon_1e6, double course_deg,
                          double long_m, double lat_m)
{
    // Everything from here to the final lround() is done in double
    // precision and rounded exactly once. peer_lat_1e6/lon are int32 fixed
    // point, course_deg comes from an int16 tenths-of-a-degree field, and
    // long_m/lat_m are configured meters — rounding any of those
    // individually before combining them would throw away precision that
    // the final result can't get back.
    double th = radians(course_deg);
    double north_m = long_m * cos(th) - lat_m * sin(th); // ahead-unit + right-unit
    double east_m  = long_m * sin(th) + lat_m * cos(th);

    double distance_m = sqrt(north_m * north_m + east_m * east_m);
    double bearing_deg = degrees(atan2(east_m, north_m));
    if (bearing_deg < 0.0)
    {
        bearing_deg += 360.0;
    }

    GNSSLocation origin{};
    origin.lat = (double)peer_lat_1e6 / 1e6;
    origin.lon = (double)peer_lon_1e6 / 1e6;

    // Reuse the existing, already-proven great-circle projection rather than
    // hand-rolling a flat-earth approximation.
    GNSSLocation projected = GNSSManager::calculatePointAtDistance(origin, distance_m, bearing_deg);

    FollowTarget target;
    target.lat_1e7 = (int32_t)lround(projected.lat * 1e7);
    target.lon_1e7 = (int32_t)lround(projected.lon * 1e7);
    return target;
}

FollowManager *followManager = nullptr;

FollowManager *FollowManager::getSingleton()
{
    if (followManager == nullptr)
    {
        followManager = new FollowManager();
        followManager->loadFromEEPROM();
    }
    return followManager;
}

bool FollowManager::followSwitchActive()
{
    switch (FOLLOW_TRIGGER_MODE)
    {
        case FOLLOW_TRIGGER_GCSNAV:
            return MSPManager::getSingleton()->isGCSNavActive();
        case FOLLOW_TRIGGER_AUX:
        default:
            // AUX-channel trigger is a later phase (spec §5[C] option 1, not
            // yet implemented) — never active rather than silently defaulting on.
            return false;
    }
}

const peer_t *FollowManager::resolveLock()
{
    PeerManager *peerManager = PeerManager::getSingleton();

    if (state == FOLLOW_LOCK_IDLE)
    {
        state = FOLLOW_LOCK_ACQUIRING;
    }

    if (state == FOLLOW_LOCK_ACQUIRING)
    {
        const peer_t *candidate = nullptr;

        if (config.targetPeer != 0)
        {
            const peer_t *p = peerManager->getPeerById(config.targetPeer);
            if (p != nullptr && !peer_is_stale(p, config.peerTimeoutMs))
            {
                candidate = p;
            }
        }
        else
        {
            for (uint8_t i = 0; i < NODES_MAX; i++)
            {
                const peer_t *p = peerManager->getPeer(i);
                if (p != nullptr && p->id > 0 && !peer_is_stale(p, config.peerTimeoutMs))
                {
                    candidate = p;
                    break;
                }
            }
        }

        if (candidate == nullptr)
        {
            return nullptr; // still acquiring
        }

        lockedId = candidate->id;
        strncpy(lockedName, candidate->name, sizeof(lockedName) - 1);
        lockedName[sizeof(lockedName) - 1] = '\0';
        state = FOLLOW_LOCK_LOCKED;
        return candidate;
    }

    if (state == FOLLOW_LOCK_LOCKED)
    {
        const peer_t *p = peerManager->getPeerById(lockedId);
        if (p == nullptr || peer_is_stale(p, config.peerTimeoutMs))
        {
            state = FOLLOW_LOCK_LOCKED_HOLDING;
            return nullptr;
        }
        return p;
    }

    // FOLLOW_LOCK_LOCKED_HOLDING: keep checking the same locked id for
    // freshness every cycle, but never scan for or switch to another peer
    // (spec §6.3 — no automatic failover).
    const peer_t *p = peerManager->getPeerById(lockedId);
    if (p != nullptr && !peer_is_stale(p, config.peerTimeoutMs))
    {
        if (strncmp(p->name, lockedName, sizeof(lockedName)) == 0)
        {
            state = FOLLOW_LOCK_LOCKED;
            return p;
        }
        // The LoRa slot id was reassigned to a different aircraft while we
        // were holding (spec §6.3 caveat). Treat as a lost lock: clear the
        // id so this can never resume, rather than silently following a
        // new aircraft under the old id. Only a gate cycle (switch off/on)
        // can recover from here.
        lockedId = 0;
    }
    return nullptr;
}

void FollowManager::forceReacquire()
{
    state = FOLLOW_LOCK_ACQUIRING;
    lockedId = 0;
    lockedName[0] = '\0';
}

double FollowManager::resolveCourseDeg(const peer_t *peer)
{
    // peer->gps.groundSpeed is int16 cm/s; minCourseSpeed is human-facing
    // m/s. Convert at the comparison site — comparing the raw values
    // directly (e.g. "200 < 2") would almost never trip and would silently
    // defeat this fallback (spec §7.5).
    int16_t minSpeedCmS = (int16_t)lround(config.minCourseSpeed * 100.0);

    if (peer->gps.groundSpeed >= minSpeedCmS)
    {
        haveValidCourse = true;
        lastValidCourseDeg = (double)peer->gps.groundCourse / 10.0;
        return lastValidCourseDeg;
    }

    // Leader's below-threshold — hold the last known course rather than
    // letting the slot swing on GPS course jitter while stationary.
    if (haveValidCourse)
    {
        return lastValidCourseDeg;
    }
    // No valid course captured yet (e.g. leader has been stationary since
    // before acquire) — fall back to whatever's reported rather than an
    // arbitrary 0.
    return (double)peer->gps.groundCourse / 10.0;
}

int16_t FollowManager::resolveHeadingDeg(const peer_t *peer, double courseDeg) const
{
    double raw;
    switch (config.headingMode)
    {
        case FOLLOW_HEADING_COURSE:
            raw = courseDeg;
            break;
        case FOLLOW_HEADING_POINT_LEADER:
        {
            // peer->gps.lat/lon are int32 x1e6 (spec §5[A] scaling note) —
            // same conversion slotToLatLon() uses for the leader's own
            // origin point.
            GNSSLocation leaderLoc{};
            leaderLoc.lat = (double)peer->gps.lat / 1e6;
            leaderLoc.lon = (double)peer->gps.lon / 1e6;
            raw = (double)GNSSManager::getSingleton()->courseTo(leaderLoc);
            break;
        }
        case FOLLOW_HEADING_FIXED:
            raw = config.headingDeg;
            break;
        case FOLLOW_HEADING_COURSE_RELATIVE:
            raw = courseDeg + config.headingDeg;
            break;
        case FOLLOW_HEADING_OFF:
        default:
            return 0; // wire sentinel: don't update heading this cycle
    }

    int32_t deg = (int32_t)lround(raw) % 360;
    if (deg < 0)
    {
        deg += 360;
    }
    if (deg == 0)
    {
        // 0 is this function's own "don't update heading" wire sentinel (the
        // default: case above) — a computed heading that legitimately wraps
        // to exactly 0/360 (due north) would collide with it, so nudge to 1°
        // instead. Imperceptible in flight. This also happens to keep the
        // value inside WP#255's p1 range (INAV's setWaypoint() only applies
        // p1 when 0 < p1 < 360, both ends exclusive) — currently moot since
        // p1 is inert for a follower in NAV POSHOLD_3D (see
        // MSPManager::sendFollowWaypoint()'s comment), but harmless to keep
        // valid there too against the day that changes. MSP_SET_HEAD itself
        // has no such range restriction.
        deg = 1;
    }
    return (int16_t)deg;
}

// Shared by targetSane() (runtime, has a live peer) and applyConfig()
// (server-side §7.4 validation of a candidate config with no peer in
// scope yet) so both always agree on the same two geometry rules. Only the
// two checks that depend purely on the configured offset — not on a live
// peer/target — live here; targetSane()'s max-distance-from-self check has
// no equivalent at config-validation time and stays in targetSane() below.
static bool offsetGeometrySane(const FollowOffset &offset, double minSepM, double minVSepM, String *errMsg)
{
    double horizontalMag = sqrt(offset.longitudinal_m * offset.longitudinal_m +
                                 offset.lateral_m * offset.lateral_m);
    double mag3d = sqrt(horizontalMag * horizontalMag + offset.vertical_m * offset.vertical_m);

    // Minimum 3D separation — forbids the degenerate collision slot (spec §7.4).
    if (mag3d < minSepM)
    {
        if (errMsg) *errMsg = "slot magnitude is below minSepM (spec §7.4 minimum 3D separation)";
        return false;
    }
    // Minimum vertical gap for stacked (overhead/underneath) slots — absorbs
    // GPS vertical error, not just physical clearance (spec §7.4).
    if (horizontalMag < FOLLOW_STACKED_HORIZONTAL_EPSILON_M && fabs(offset.vertical_m) < minVSepM)
    {
        if (errMsg) *errMsg = "stacked slot's vertical offset is below minVSepM (spec §7.4)";
        return false;
    }
    return true;
}

// Layer 2 of spec §4: true if `axis` crossing from `referenceAxis`'s sign
// to `candidateAxis`'s sign is unsafe right now. Only a genuine sign flip
// counts as "crossing" (spec §4.3 condition 1) — 0 on either side is the
// boundary itself, not a side, so it never counts as a flip. `coMag` is
// the *smaller* of the other two axes' combined magnitude at the
// reference point and at the candidate point (spec §4.3's conservative
// choice, covering two RC-assigned axes swinging in the same cycle).
static bool axisSignLocked(double candidateAxis, double referenceAxis,
                            double candidateOther1, double candidateOther2,
                            double referenceOther1, double referenceOther2,
                            double minSepM)
{
    bool crossed = (candidateAxis > 0 && referenceAxis < 0) ||
                   (candidateAxis < 0 && referenceAxis > 0);
    if (!crossed)
    {
        return false;
    }
    double coMagCandidate = sqrt(candidateOther1 * candidateOther1 + candidateOther2 * candidateOther2);
    double coMagReference = sqrt(referenceOther1 * referenceOther1 + referenceOther2 * referenceOther2);
    double coMag = min(coMagCandidate, coMagReference);
    return coMag < minSepM;
}

// Both safety layers (spec §4.2 Layer 1, §4.3 Layer 2), evaluated
// together so there's exactly one pass/fail test. Shared, read-only,
// between resolveOffset() (which adopts `candidate` as the new
// lastKnownGood on a pass) and the §4.6 pre-arm advisory check (which
// never mutates state) — both callers must always agree on the same
// answer for the same inputs.
static bool candidateOffsetOk(const FollowOffset &candidate, const FollowOffset &reference,
                               double minSepM, double minVSepM)
{
    if (!offsetGeometrySane(candidate, minSepM, minVSepM, nullptr))
    {
        return false;
    }
    if (axisSignLocked(candidate.longitudinal_m, reference.longitudinal_m,
                        candidate.lateral_m, candidate.vertical_m,
                        reference.lateral_m, reference.vertical_m, minSepM))
    {
        return false;
    }
    if (axisSignLocked(candidate.lateral_m, reference.lateral_m,
                        candidate.longitudinal_m, candidate.vertical_m,
                        reference.longitudinal_m, reference.vertical_m, minSepM))
    {
        return false;
    }
    if (axisSignLocked(candidate.vertical_m, reference.vertical_m,
                        candidate.longitudinal_m, candidate.lateral_m,
                        reference.longitudinal_m, reference.lateral_m, minSepM))
    {
        return false;
    }
    return true;
}

// Pre-arm-only, stricter than Layer 1/Layer 2 above: requires every
// RC-assigned axis's resolved candidate to exactly reproduce the saved
// static default, not merely avoid the collision-risk geometry those
// layers guard against. The pilot's stick must sit at the
// one position (full deflection toward the default's sign, since
// resolveAxisOffset() maps center to 0) that reproduces the configured
// value, or the pre-arm warning stays lit. Axes with no RC channel
// assigned always match trivially, since resolveAxisOffset() returns
// configuredM verbatim for them.
static bool rcCandidateMatchesStaticDefault(const FollowOffset &candidate, const FollowRuntimeConfig &config)
{
    if (config.rcLongChannel != -1 &&
        fabs(candidate.longitudinal_m - config.ofsLongM) > FOLLOW_PREARM_MATCH_EPSILON_M)
    {
        return false;
    }
    if (config.rcLatChannel != -1 &&
        fabs(candidate.lateral_m - config.ofsLatM) > FOLLOW_PREARM_MATCH_EPSILON_M)
    {
        return false;
    }
    if (config.rcVertChannel != -1 &&
        fabs(candidate.vertical_m - config.ofsVertM) > FOLLOW_PREARM_MATCH_EPSILON_M)
    {
        return false;
    }
    return true;
}

double FollowManager::resolveAxisOffset(double configuredM, int16_t channel1Based) const
{
    if (channel1Based < 1)
    {
        return configuredM; // no channel assigned (spec §3.2)
    }

    uint16_t us;
    if (!MSPManager::getSingleton()->getRcChannelUs((uint8_t)channel1Based, &us))
    {
        return configuredM; // no FC connected, or channel1Based out of MSP_RC's range (spec §3.2)
    }
    // Whatever value comes back is mapped as-is, including below 1000us --
    // there is no separate "invalid reading" case once a channel is
    // assigned (spec §3.2). The clamp below already guards over/under-
    // travel, so this also covers an unpopulated channel index (reads 0us
    // from the cached msp_rc_t, spec §2.1) the same way: it resolves to
    // -gap, not a fallback to configuredM.

    double gap = fabs(configuredM);
    uint16_t usClamped = constrain(us, 1000, 2000);
    double frac = ((double)usClamped - 1500.0) / 500.0; // -1.0 .. +1.0
    return frac * gap;
}

bool FollowManager::anyRcChannelAssigned() const
{
    return config.rcLongChannel != -1 || config.rcLatChannel != -1 || config.rcVertChannel != -1;
}

FollowOffset FollowManager::resolveCandidateOffset() const
{
    return {
        resolveAxisOffset(config.ofsLongM, config.rcLongChannel),
        resolveAxisOffset(config.ofsLatM, config.rcLatChannel),
        resolveAxisOffset(config.ofsVertM, config.rcVertChannel),
    };
}

FollowOffset FollowManager::resolveOffset()
{
    FollowOffset candidate = resolveCandidateOffset();

    bool ok = candidateOffsetOk(candidate, lastKnownGood, config.minSepM, config.minVSepM);
    rcSlotFrozen = !ok;
    if (ok)
    {
        lastKnownGood = candidate;
    }
    // On failure, lastKnownGood is left exactly as it was — the freeze
    // (spec §4.4). When no axis has an RC channel assigned, `candidate`
    // always equals lastKnownGood already (both are the static config,
    // which applyConfig() guarantees is geometry-sane), so ok is always
    // true and this is a no-op — today's plain static-offset behavior,
    // unchanged.
    return lastKnownGood;
}

bool FollowManager::targetSane(const FollowOffset &offset, const FollowTarget &target)
{
    if (!offsetGeometrySane(offset, config.minSepM, config.minVSepM, nullptr))
    {
        return false;
    }

    // Runtime sanity: the solved target shouldn't be unreasonably far from
    // the follower's own position (spec §7.4).
    GNSSLocation targetLoc{};
    targetLoc.lat = (double)target.lat_1e7 / 1e7;
    targetLoc.lon = (double)target.lon_1e7 / 1e7;
    double distFromSelf = GNSSManager::getSingleton()->horizontalDistanceTo(targetLoc);
    if (distFromSelf > config.maxTargetDistM)
    {
        return false;
    }

    return true;
}

void FollowManager::loop()
{
    if (sys.phase <= MODE_OTA_SYNC)
    {
        return;
    }
    if (millis() < nextRunTime)
    {
        return;
    }
    nextRunTime = millis() + (1000 / config.emitHz);

    // spec §4.6: catch the common "RC disagrees with the static default's
    // sign" bootstrap trap (spec §10) while still on the ground, where the
    // pilot can simply move the stick before it matters mid-flight. Runs
    // independent of the follow gate/GCS NAV (spec §4.6 — the point is to
    // catch this before the pilot ever reaches for the switch, not just
    // while follow is inactive), gated only on arm state so it can't freeze
    // stale if the gate goes active before the pilot arms. Reset to false
    // every cycle the craft isn't disarmed-with-an-axis-assigned, so it
    // never reports stale while armed (spec §7's gating for rcPreArmCheckFailed).
    rcPreArmCheckFailed = false;
    havePreArmCandidateOffset = false;
    if (MSPManager::getSingleton()->getState() == 0 && anyRcChannelAssigned())
    {
        // Read-only: deliberately does not touch lastKnownGood (spec §4.6's
        // "never write to lastKnownGood" requirement) — this is a
        // simulation of "what would happen if follow engaged right now,"
        // not a real state transition.
        preArmCandidateOffset = resolveCandidateOffset();
        havePreArmCandidateOffset = true;
        // Two independent reasons to warn: the collision-geometry check
        // (Layer 1/2) can still pass on a sign-flipped axis if the other
        // two axes already clear minSepM on their own, so it alone doesn't
        // guarantee RC agrees with the static default — the exact-match
        // check below closes that gap.
        rcPreArmCheckFailed = !candidateOffsetOk(preArmCandidateOffset, lastKnownGood, config.minSepM, config.minVSepM) ||
                              !rcCandidateMatchesStaticDefault(preArmCandidateOffset, config);
    }

    if (!followSwitchActive())
    {
        state = FOLLOW_LOCK_IDLE;
        lockedId = 0;
        lockedName[0] = '\0';
        haveValidCourse = false;
        updateStatusGvars(rcPreArmCheckFailed ? FOLLOW_CONDITION_RC_INVALID_GAP_SETTINGS : FOLLOW_CONDITION_NONE);
        return;
    }

    const peer_t *peer = resolveLock();
    if (peer == nullptr)
    {
        updateStatusGvars(FOLLOW_CONDITION_NONE);
        return; // ACQUIRING or LOCKED_HOLDING this cycle — nothing to emit
    }

    FollowOffset offset = resolveOffset();
    double courseDeg = resolveCourseDeg(peer);

    FollowTarget target = slotToLatLon(peer->gps.lat, peer->gps.lon, courseDeg,
                                        offset.longitudinal_m, offset.lateral_m);

    // [B] local_altitude_cm() is the follower's baro/GPS-fused home-relative
    // estimate; peer->relalt is a raw-GPS-only delta (leader minus
    // follower). Summed here in double precision and rounded once, since
    // relalt*100 is exact but offset.vertical_m*100 generally isn't
    // (spec §6.2 — this frame mixing is a known accuracy bound, not a bug;
    // see FOLLOW_MIN_VSEP_M's GPS-error margin).
    double altCmD = (double)MSPManager::getSingleton()->local_altitude_cm()
                   + (double)peer->relalt * 100.0
                   + offset.vertical_m * 100.0;
    int32_t altCm = (int32_t)lround(altCmD);

    // Hard floor (spec §7.6): never command the follower below a
    // configurable minimum home-relative altitude, regardless of what the
    // sum above produced — e.g. the leader flying low/landing, or a BELOW
    // slot dragging the follower toward the ground. Clamp, don't reject:
    // unlike targetSane() below, this must not suppress the waypoint —
    // the follower should keep tracking laterally and hold at the floor.
    int32_t floorCm = (int32_t)lround(config.minAltM * 100.0);
    bool floorClamped = altCm < floorCm;
    if (floorClamped)
    {
        altCm = floorCm;
    }

    // spec §5.1: attribute the clamp to RC only if the plain configured
    // (non-RC-scaled) vertical offset would NOT also have clamped. When no
    // channel is assigned to vertical, offset.vertical_m == config.ofsVertM
    // by construction (§3.2), so this always agrees with the "actual" check
    // and degrades to today's plain floor-clamp behavior with no special-casing.
    bool floorAttributableToRc = false;
    if (floorClamped)
    {
        // Reuses altCmD's already-computed local_altitude_cm()+relalt sum,
        // just swapping in the static (non-RC-scaled) vertical offset,
        // instead of re-deriving the whole sum from scratch.
        double altCmStaticD = altCmD + (config.ofsVertM - offset.vertical_m) * 100.0;
        int32_t altCmStatic = (int32_t)lround(altCmStaticD);
        floorAttributableToRc = altCmStatic >= floorCm;
    }

    // spec §5.3's condition-code table: code 2 whenever either mechanism is
    // active (they never disagree — see spec §5.2), otherwise code 1 if just
    // the floor clamped for a reason unrelated to RC, otherwise 0.
    FollowConditionCode conditionCode = FOLLOW_CONDITION_NONE;
    if (floorClamped)
    {
        conditionCode = floorAttributableToRc ? FOLLOW_CONDITION_RC_INVALID_GAP_SETTINGS : FOLLOW_CONDITION_FLOOR_CLAMPED;
    }
    if (rcSlotFrozen)
    {
        conditionCode = FOLLOW_CONDITION_RC_INVALID_GAP_SETTINGS;
    }

    if (!targetSane(offset, target))
    {
        updateStatusGvars(conditionCode);
        return;
    }

    // Nose heading (spec §7.7) — independent of the position target above.
    // Computed the same way regardless of follower airframe; no craft-type
    // branch (spec §7.7 explains why that's safe for fixed-wing too).
    int16_t headingDeg = resolveHeadingDeg(peer, courseDeg);

    updateDebugGvars(target.lat_1e7, target.lon_1e7, altCm, headingDeg);

    MSPManager *msp = MSPManager::getSingleton();
    // headingDeg is also passed to sendFollowWaypoint() below (WP#255's p1) —
    // currently inert on INAV 9.x for a follower in NAV POSHOLD_3D, kept as a
    // forward-compatible best-effort write (see that function's comment).
    // sendSetHead() (MSP_SET_HEAD) is the mechanism that actually works today.
    msp->sendFollowWaypoint(target.lat_1e7, target.lon_1e7, altCm, headingDeg);
    // headingDeg == 0 is FOLLOW_HEADING_OFF's wire sentinel (resolveHeadingDeg()
    // never returns 0 for any other mode — see its comment) — skip sending in
    // that case rather than commanding due north. isHeadingHoldActive() gates
    // on INAV's own HEADING HOLD box being active, the precondition for this
    // target to actually reach the yaw-rate PID while in NAV POSHOLD_3D
    // (see MSPManager::sendSetHead()'s comment for why).
    if (headingDeg != 0 && msp->isHeadingHoldActive())
    {
        msp->sendSetHead(headingDeg);
    }
    updateStatusGvars(conditionCode);

    haveLastTarget = true;
    lastTarget = target;
    lastTargetAltCm = altCm;
    lastTargetHeadingDeg = headingDeg;
    lastTargetTime = millis();
    lastLiveOffset = offset; // spec §7 liveOffset
}

static const char *lockStateName(FollowLockState state)
{
    switch (state)
    {
        case FOLLOW_LOCK_IDLE:            return "IDLE";
        case FOLLOW_LOCK_ACQUIRING:       return "ACQUIRING";
        case FOLLOW_LOCK_LOCKED:          return "LOCKED";
        case FOLLOW_LOCK_LOCKED_HOLDING:  return "LOCKED_HOLDING";
        default:                          return "UNKNOWN";
    }
}

void FollowManager::statusJson(JsonDocument *doc)
{
    (*doc)["state"] = lockStateName(state);
    (*doc)["gateActive"] = followSwitchActive();
    (*doc)["lockedId"] = lockedId;
    (*doc)["lockedName"] = lockedName;
    if (haveLastTarget)
    {
        JsonObject target = doc->createNestedObject("lastTarget");
        target["lat"] = lastTarget.lat_1e7;
        target["lon"] = lastTarget.lon_1e7;
        target["altCm"] = lastTargetAltCm;
        target["headingDeg"] = lastTargetHeadingDeg;
        target["ageMs"] = millis() - lastTargetTime;
    }
    if (config.statusGvarIndex >= 0 && lastSentStatusGvarValue != INT32_MIN)
    {
        (*doc)["statusGvarValue"] = lastSentStatusGvarValue;
    }
    if (config.conditionFlagsGvarIndex >= 0 && lastSentConditionFlagsGvarValue != INT32_MIN)
    {
        (*doc)["conditionFlagsGvarValue"] = lastSentConditionFlagsGvarValue;
    }
    if (haveLastTarget)
    {
        JsonObject live = doc->createNestedObject("liveOffset");
        live["longM"] = lastLiveOffset.longitudinal_m;
        live["latM"] = lastLiveOffset.lateral_m;
        live["vertM"] = lastLiveOffset.vertical_m;
        (*doc)["rcSlotFrozen"] = rcSlotFrozen;
    }
    if (havePreArmCandidateOffset)
    {
        JsonObject preArm = doc->createNestedObject("preArmCandidateOffset");
        preArm["longM"] = preArmCandidateOffset.longitudinal_m;
        preArm["latM"] = preArmCandidateOffset.lateral_m;
        preArm["vertM"] = preArmCandidateOffset.vertical_m;
    }
    (*doc)["rcPreArmCheckFailed"] = rcPreArmCheckFailed;
}

// Spec §3's status code. IDLE only appears transiently (loop() sets it
// right before the gate-inactive early return) — included for
// completeness, not reachable with a nonzero code.
static int32_t statusGvarValue(FollowLockState state, uint8_t lockedId)
{
    switch (state)
    {
        case FOLLOW_LOCK_ACQUIRING:      return 1;
        case FOLLOW_LOCK_LOCKED:         return 2;
        // lockedId == 0 only happens here via the id-reuse-mismatch path
        // in resolveLock() (spec §6.3 caveat of the parent spec) — see
        // this plan's "ID LOST" decision above.
        case FOLLOW_LOCK_LOCKED_HOLDING: return lockedId == 0 ? 4 : 3;
        case FOLLOW_LOCK_IDLE:
        default:                          return 0;
    }
}

void FollowManager::updateStatusGvars(FollowConditionCode conditionCode)
{
    MSPManager *msp = MSPManager::getSingleton();
    unsigned long now = millis();

    if (config.statusGvarIndex >= 0)
    {
        int32_t value = statusGvarValue(state, lockedId);
        bool due = lastSentStatusGvarValue == INT32_MIN
                 || value != lastSentStatusGvarValue
                 || (now - lastStatusGvarSendMs) >= FOLLOW_GVAR_HEARTBEAT_MS;
        if (due)
        {
            msp->sendGvar((uint8_t)config.statusGvarIndex, value);
            lastSentStatusGvarValue = value;
            lastStatusGvarSendMs = now;
        }
    }

    if (config.conditionFlagsGvarIndex >= 0)
    {
        int32_t value = conditionCode; // spec §5.3's 0/1/2 table, computed by callers now
        bool due = lastSentConditionFlagsGvarValue == INT32_MIN
                 || value != lastSentConditionFlagsGvarValue
                 || (now - lastConditionFlagsGvarSendMs) >= FOLLOW_GVAR_HEARTBEAT_MS;
        if (due)
        {
            msp->sendGvar((uint8_t)config.conditionFlagsGvarIndex, value);
            lastSentConditionFlagsGvarValue = value;
            lastConditionFlagsGvarSendMs = now;
        }
    }
}

// lat_1e7/lon_1e7 are the follower's commanded waypoint (x1e7 degrees, same
// as FollowTarget), altCm is home-relative cm, headingDeg is whole degrees —
// the exact values just passed to sendFollowWaypoint(), sent as-is since this
// is a raw debug readout, not a value the OSD needs scaled for display.
void FollowManager::updateDebugGvars(int32_t lat_1e7, int32_t lon_1e7, int32_t altCm, int16_t headingDeg)
{
    if (!config.debug)
    {
        return;
    }
    MSPManager *msp = MSPManager::getSingleton();
    msp->sendGvar(FOLLOW_DEBUG_LAT_GVAR_INDEX, lat_1e7);
    msp->sendGvar(FOLLOW_DEBUG_LON_GVAR_INDEX, lon_1e7);
    msp->sendGvar(FOLLOW_DEBUG_ALT_GVAR_INDEX, altCm);
    msp->sendGvar(FOLLOW_DEBUG_HEADING_GVAR_INDEX, headingDeg);
}

static const char *headingModeName(FollowHeadingMode m)
{
    switch (m)
    {
        case FOLLOW_HEADING_COURSE:          return "COURSE";
        case FOLLOW_HEADING_POINT_LEADER:    return "POINT_LEADER";
        case FOLLOW_HEADING_FIXED:           return "FIXED";
        case FOLLOW_HEADING_COURSE_RELATIVE: return "COURSE_RELATIVE";
        case FOLLOW_HEADING_OFF:
        default:                              return "OFF";
    }
}

static const char *triggerModeName(FollowTriggerMode m)
{
    switch (m)
    {
        case FOLLOW_TRIGGER_AUX: return "AUX";
        case FOLLOW_TRIGGER_GCSNAV:
        default:                  return "GCSNAV";
    }
}

void FollowManager::configJson(JsonDocument *doc) const
{
    (*doc)["ofsLongM"] = config.ofsLongM;
    (*doc)["ofsLatM"] = config.ofsLatM;
    (*doc)["ofsVertM"] = config.ofsVertM;

    // Trigger mode is compile-time-only until Phase 2b (AUX) lands — report
    // it read-only rather than accepting it via applyConfig() (spec plan's
    // Phase 3 notes).
    (*doc)["triggerMode"] = triggerModeName((FollowTriggerMode)FOLLOW_TRIGGER_MODE);

    (*doc)["targetPeer"] = config.targetPeer;
    (*doc)["emitHz"] = config.emitHz;
    (*doc)["peerTimeoutMs"] = config.peerTimeoutMs;

    (*doc)["minSepM"] = config.minSepM;
    (*doc)["minVSepM"] = config.minVSepM;
    (*doc)["maxTargetDistM"] = config.maxTargetDistM;
    (*doc)["minAltM"] = config.minAltM;

    (*doc)["minCourseSpeed"] = config.minCourseSpeed;

    (*doc)["headingMode"] = headingModeName(config.headingMode);
    (*doc)["headingDeg"] = config.headingDeg;

    (*doc)["statusGvarIndex"] = config.statusGvarIndex;
    (*doc)["conditionFlagsGvarIndex"] = config.conditionFlagsGvarIndex;

    (*doc)["rcLongChannel"] = config.rcLongChannel;
    (*doc)["rcLatChannel"] = config.rcLatChannel;
    (*doc)["rcVertChannel"] = config.rcVertChannel;

    // RAM only (spec: FollowConfig.h's FOLLOW_DEBUG_ENABLED comment) — not
    // in FollowEepromRecord, so this always reports false again after a
    // reboot regardless of what was last applied.
    (*doc)["debug"] = config.debug;
}

bool FollowManager::applyConfig(const FollowRuntimeConfig &newConfig, String *errMsg)
{
    String localErr;
    if (!errMsg) errMsg = &localErr;

    if (newConfig.emitHz == 0)
    {
        *errMsg = "emitHz must be > 0";
        return false;
    }
    if (newConfig.peerTimeoutMs == 0)
    {
        *errMsg = "peerTimeoutMs must be > 0";
        return false;
    }
    if (newConfig.minSepM < 0 || newConfig.minVSepM < 0 || newConfig.minAltM < 0)
    {
        *errMsg = "minSepM/minVSepM/minAltM must be >= 0";
        return false;
    }
    if (newConfig.maxTargetDistM <= 0)
    {
        *errMsg = "maxTargetDistM must be > 0";
        return false;
    }
    if (newConfig.minCourseSpeed < 0)
    {
        *errMsg = "minCourseSpeed must be >= 0";
        return false;
    }
    if (newConfig.targetPeer > NODES_MAX)
    {
        *errMsg = "targetPeer out of range";
        return false;
    }
    if (newConfig.statusGvarIndex < -1 || newConfig.statusGvarIndex > 7)
    {
        *errMsg = "statusGvarIndex must be -1 (disabled) or 0-7";
        return false;
    }
    if (newConfig.conditionFlagsGvarIndex < -1 || newConfig.conditionFlagsGvarIndex > 7)
    {
        *errMsg = "conditionFlagsGvarIndex must be -1 (disabled) or 0-7";
        return false;
    }
    if (newConfig.rcLongChannel != -1 && (newConfig.rcLongChannel < 1 || newConfig.rcLongChannel > MSP_MAX_SUPPORTED_CHANNELS))
    {
        *errMsg = "rcLongChannel must be -1 (disabled) or 1-16";
        return false;
    }
    if (newConfig.rcLatChannel != -1 && (newConfig.rcLatChannel < 1 || newConfig.rcLatChannel > MSP_MAX_SUPPORTED_CHANNELS))
    {
        *errMsg = "rcLatChannel must be -1 (disabled) or 1-16";
        return false;
    }
    if (newConfig.rcVertChannel != -1 && (newConfig.rcVertChannel < 1 || newConfig.rcVertChannel > MSP_MAX_SUPPORTED_CHANNELS))
    {
        *errMsg = "rcVertChannel must be -1 (disabled) or 1-16";
        return false;
    }

    // Spec §7.4 geometry rules, evaluated against the config's canonical
    // offset — mirrors targetSane()'s two config-only checks so a config
    // that's accepted here can never be rejected by targetSane() for the
    // same reason later.
    FollowOffset offset = { newConfig.ofsLongM, newConfig.ofsLatM, newConfig.ofsVertM };
    if (!offsetGeometrySane(offset, newConfig.minSepM, newConfig.minVSepM, errMsg))
    {
        return false;
    }

    bool targetPeerChanged = (newConfig.targetPeer != config.targetPeer);
    config = newConfig;
    // §4.4: a config change (new gaps, a reassigned channel) can make the
    // previously-frozen triple meaningless, so re-anchor it to the new
    // static offset — the one point applyConfig() already guarantees is
    // geometry-sane via the offsetGeometrySane() check above.
    lastKnownGood = { config.ofsLongM, config.ofsLatM, config.ofsVertM };
    if (targetPeerChanged)
    {
        forceReacquire();
    }
    return true;
}

// Converts a runtime config to its narrower on-disk form (see
// FollowEepromRecord's comment in FollowManager.h for why int16_t loses no
// precision here). lround(), not a plain cast, so a value the UI couldn't
// have produced anyway (e.g. one seeded from a compile-time #define with a
// fractional value) rounds to the nearest representable integer instead of
// silently truncating toward zero.
static FollowEepromRecord toEepromRecord(const FollowRuntimeConfig &config)
{
    FollowEepromRecord record{};
    record.version = FOLLOW_EEPROM_VERSION;

    record.ofsLongM = (int16_t)lround(config.ofsLongM);
    record.ofsLatM = (int16_t)lround(config.ofsLatM);
    record.ofsVertM = (int16_t)lround(config.ofsVertM);

    record.targetPeer = config.targetPeer;
    record.emitHz = config.emitHz;
    record.peerTimeoutMs = config.peerTimeoutMs;

    record.minSepM = (int16_t)lround(config.minSepM);
    record.minVSepM = (int16_t)lround(config.minVSepM);
    record.maxTargetDistM = (int16_t)lround(config.maxTargetDistM);
    record.minAltM = (int16_t)lround(config.minAltM);

    record.minCourseSpeed = (int16_t)lround(config.minCourseSpeed);

    record.headingMode = config.headingMode;
    record.headingDeg = (int16_t)lround(config.headingDeg);

    record.statusGvarIndex = config.statusGvarIndex;
    record.conditionFlagsGvarIndex = config.conditionFlagsGvarIndex;

    record.rcLongChannel = config.rcLongChannel;
    record.rcLatChannel = config.rcLatChannel;
    record.rcVertChannel = config.rcVertChannel;

    return record;
}

// Converts the on-disk record back to a runtime config. The int16_t->double
// widening here is always exact (every integer int16_t can represent is
// exactly representable as a double), so unlike toEepromRecord() above this
// direction has no rounding to reason about.
static FollowRuntimeConfig fromEepromRecord(const FollowEepromRecord &record)
{
    FollowRuntimeConfig config;

    config.ofsLongM = record.ofsLongM;
    config.ofsLatM = record.ofsLatM;
    config.ofsVertM = record.ofsVertM;

    config.targetPeer = record.targetPeer;
    config.emitHz = record.emitHz;
    config.peerTimeoutMs = record.peerTimeoutMs;

    config.minSepM = record.minSepM;
    config.minVSepM = record.minVSepM;
    config.maxTargetDistM = record.maxTargetDistM;
    config.minAltM = record.minAltM;

    config.minCourseSpeed = record.minCourseSpeed;

    config.headingMode = record.headingMode;
    config.headingDeg = record.headingDeg;

    config.statusGvarIndex = record.statusGvarIndex;
    config.conditionFlagsGvarIndex = record.conditionFlagsGvarIndex;

    config.rcLongChannel = record.rcLongChannel;
    config.rcLatChannel = record.rcLatChannel;
    config.rcVertChannel = record.rcVertChannel;

    return config;
}

void FollowManager::loadFromEEPROM()
{
    FollowEepromRecord record;
    EEPROM.get(FOLLOW_EEPROM_OFFSET, record);
    if (record.version != FOLLOW_EEPROM_VERSION)
    {
        // Fresh flash / nothing saved yet — keep the compile-time defaults
        // FollowRuntimeConfig's member initializers already seeded.
        return;
    }
    // Reuse applyConfig()'s validation (spec §7.4 + basic field sanity) so
    // a corrupted or stale-schema record (bit flips, a struct layout that
    // changed since it was written) can't silently arm follow with insane
    // geometry. forceReacquire() is a no-op here — no peer lock exists yet
    // at boot.
    String errMsg;
    if (!applyConfig(fromEepromRecord(record), &errMsg))
    {
        DBGF("[FollowManager] ignoring invalid EEPROM config: %s\n", errMsg.c_str());
    }
}

bool FollowManager::saveToEEPROM(String *errMsg)
{
    String localErr;
    if (!errMsg) errMsg = &localErr;

    unsigned long now = millis();
    if (lastEepromCommitMs != 0 && now - lastEepromCommitMs < FOLLOW_EEPROM_COMMIT_MIN_INTERVAL_MS)
    {
        *errMsg = "saved too recently, try again shortly";
        return false;
    }

    FollowEepromRecord record = toEepromRecord(config);
    EEPROM.put(FOLLOW_EEPROM_OFFSET, record);
    EEPROM.commit();
    lastEepromCommitMs = now;
    return true;
}
