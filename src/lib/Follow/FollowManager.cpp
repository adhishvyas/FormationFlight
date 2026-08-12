#include "FollowManager.h"
#include "../MSP/MSPManager.h"
#include "../GNSS/GNSSManager.h"
#include "main.h"
#include <math.h>

// Below this horizontal offset magnitude, a slot is considered "stacked"
// (overhead/underneath) for the purposes of the minimum-vertical-separation
// rule (spec §7.4).
#define FOLLOW_STACKED_HORIZONTAL_EPSILON_M 0.5

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

FollowOffset FollowManager::offsetFromConfig(const FollowRuntimeConfig &cfg) const
{
    if (cfg.offsetMode == FOLLOW_OFFSET_MODE_RAW)
    {
        return { cfg.ofsLongM, cfg.ofsLatM, cfg.ofsVertM };
    }

    FollowOffset o{};

    switch (cfg.slotLong)
    {
        case FOLLOW_LONG_AHEAD:  o.longitudinal_m = cfg.gapLongM; break;
        case FOLLOW_LONG_BEHIND: o.longitudinal_m = -cfg.gapLongM; break;
        case FOLLOW_LONG_CENTER:
        default:                 o.longitudinal_m = 0.0; break;
    }

    switch (cfg.slotLat)
    {
        case FOLLOW_LAT_RIGHT: o.lateral_m = cfg.gapLatM; break;
        case FOLLOW_LAT_LEFT:  o.lateral_m = -cfg.gapLatM; break;
        case FOLLOW_LAT_CENTER:
        default:                o.lateral_m = 0.0; break;
    }

    switch (cfg.slotVert)
    {
        case FOLLOW_VERT_ABOVE: o.vertical_m = cfg.gapVertM; break;
        case FOLLOW_VERT_BELOW: o.vertical_m = -cfg.gapVertM; break;
        case FOLLOW_VERT_LEVEL:
        default:                 o.vertical_m = 0.0; break;
    }

    return o;
}

FollowOffset FollowManager::resolveOffset()
{
    return offsetFromConfig(config);
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

    switch (config.stationaryMode)
    {
        case FOLLOW_STATIONARY_HOLD_COURSE:
            if (haveValidCourse)
            {
                return lastValidCourseDeg;
            }
            // No valid course captured yet (e.g. leader has been stationary
            // since before acquire) — fall back to whatever's reported
            // rather than an arbitrary 0.
            return (double)peer->gps.groundCourse / 10.0;
        case FOLLOW_STATIONARY_WORLD_FRAME:
        default:
            // Fixed world-frame axes: longitudinal = north, lateral = east.
            return 0.0;
    }
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

    if (!followSwitchActive())
    {
        state = FOLLOW_LOCK_IDLE;
        lockedId = 0;
        lockedName[0] = '\0';
        haveValidCourse = false;
        return;
    }

    const peer_t *peer = resolveLock();
    if (peer == nullptr)
    {
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
    if (altCm < floorCm)
    {
        altCm = floorCm;
    }

    if (!targetSane(offset, target))
    {
        return;
    }

    MSPManager::getSingleton()->sendFollowWaypoint(target.lat_1e7, target.lon_1e7, altCm);

    haveLastTarget = true;
    lastTarget = target;
    lastTargetAltCm = altCm;
    lastTargetTime = millis();
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
        target["ageMs"] = millis() - lastTargetTime;
    }
}

static const char *offsetModeName(FollowOffsetMode m)
{
    switch (m)
    {
        case FOLLOW_OFFSET_MODE_RAW: return "RAW";
        case FOLLOW_OFFSET_MODE_GRID:
        default:                     return "GRID";
    }
}

static const char *slotLongName(FollowLongSlot s)
{
    switch (s)
    {
        case FOLLOW_LONG_AHEAD:  return "AHEAD";
        case FOLLOW_LONG_BEHIND: return "BEHIND";
        case FOLLOW_LONG_CENTER:
        default:                 return "CENTER";
    }
}

static const char *slotLatName(FollowLatSlot s)
{
    switch (s)
    {
        case FOLLOW_LAT_LEFT:  return "LEFT";
        case FOLLOW_LAT_RIGHT: return "RIGHT";
        case FOLLOW_LAT_CENTER:
        default:                return "CENTER";
    }
}

static const char *slotVertName(FollowVertSlot s)
{
    switch (s)
    {
        case FOLLOW_VERT_ABOVE: return "ABOVE";
        case FOLLOW_VERT_BELOW: return "BELOW";
        case FOLLOW_VERT_LEVEL:
        default:                 return "LEVEL";
    }
}

static const char *stationaryModeName(FollowStationaryMode m)
{
    switch (m)
    {
        case FOLLOW_STATIONARY_WORLD_FRAME: return "WORLD_FRAME";
        case FOLLOW_STATIONARY_HOLD_COURSE:
        default:                             return "HOLD_COURSE";
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
    (*doc)["offsetMode"] = offsetModeName(config.offsetMode);
    (*doc)["slotLong"] = slotLongName(config.slotLong);
    (*doc)["slotLat"] = slotLatName(config.slotLat);
    (*doc)["slotVert"] = slotVertName(config.slotVert);
    (*doc)["gapLongM"] = config.gapLongM;
    (*doc)["gapLatM"] = config.gapLatM;
    (*doc)["gapVertM"] = config.gapVertM;

    // Resolved canonical offsets regardless of mode, so the UI can show
    // "what this actually resolves to right now" even while editing the
    // grid view (spec §10.3 — "reflects grid->canonical expansion").
    FollowOffset resolved = offsetFromConfig(config);
    (*doc)["ofsLongM"] = resolved.longitudinal_m;
    (*doc)["ofsLatM"] = resolved.lateral_m;
    (*doc)["ofsVertM"] = resolved.vertical_m;

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
    (*doc)["stationaryMode"] = stationaryModeName(config.stationaryMode);
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
    if (newConfig.gapLongM < 0 || newConfig.gapLatM < 0 || newConfig.gapVertM < 0)
    {
        *errMsg = "gap values must be >= 0";
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

    // Spec §7.4 geometry rules, evaluated against what this config actually
    // resolves to (grid-expanded or raw, per offsetMode) — mirrors
    // targetSane()'s two config-only checks so a config that's accepted
    // here can never be rejected by targetSane() for the same reason later.
    FollowOffset offset = offsetFromConfig(newConfig);
    if (!offsetGeometrySane(offset, newConfig.minSepM, newConfig.minVSepM, errMsg))
    {
        return false;
    }

    bool targetPeerChanged = (newConfig.targetPeer != config.targetPeer);
    config = newConfig;
    if (targetPeerChanged)
    {
        forceReacquire();
    }
    return true;
}
