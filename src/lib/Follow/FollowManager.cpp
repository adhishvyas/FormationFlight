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

        if (FOLLOW_TARGET_PEER != 0)
        {
            const peer_t *p = peerManager->getPeerById(FOLLOW_TARGET_PEER);
            if (p != nullptr && !peer_is_stale(p, FOLLOW_PEER_TIMEOUT_MS))
            {
                candidate = p;
            }
        }
        else
        {
            for (uint8_t i = 0; i < NODES_MAX; i++)
            {
                const peer_t *p = peerManager->getPeer(i);
                if (p != nullptr && p->id > 0 && !peer_is_stale(p, FOLLOW_PEER_TIMEOUT_MS))
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
        if (p == nullptr || peer_is_stale(p, FOLLOW_PEER_TIMEOUT_MS))
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
    if (p != nullptr && !peer_is_stale(p, FOLLOW_PEER_TIMEOUT_MS))
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

FollowOffset FollowManager::resolveOffset()
{
    FollowOffset o{};

    switch (FOLLOW_SLOT_LONG)
    {
        case FOLLOW_LONG_AHEAD:  o.longitudinal_m = FOLLOW_GAP_LONG_M; break;
        case FOLLOW_LONG_BEHIND: o.longitudinal_m = -FOLLOW_GAP_LONG_M; break;
        case FOLLOW_LONG_CENTER:
        default:                 o.longitudinal_m = 0.0; break;
    }

    switch (FOLLOW_SLOT_LAT)
    {
        case FOLLOW_LAT_RIGHT: o.lateral_m = FOLLOW_GAP_LAT_M; break;
        case FOLLOW_LAT_LEFT:  o.lateral_m = -FOLLOW_GAP_LAT_M; break;
        case FOLLOW_LAT_CENTER:
        default:                o.lateral_m = 0.0; break;
    }

    switch (FOLLOW_SLOT_VERT)
    {
        case FOLLOW_VERT_ABOVE: o.vertical_m = FOLLOW_GAP_VERT_M; break;
        case FOLLOW_VERT_BELOW: o.vertical_m = -FOLLOW_GAP_VERT_M; break;
        case FOLLOW_VERT_LEVEL:
        default:                 o.vertical_m = 0.0; break;
    }

    return o;
}

double FollowManager::resolveCourseDeg(const peer_t *peer)
{
    // peer->gps.groundSpeed is int16 cm/s; FOLLOW_MIN_COURSE_SPEED is
    // human-facing m/s. Convert at the comparison site — comparing the raw
    // values directly (e.g. "200 < 2") would almost never trip and would
    // silently defeat this fallback (spec §7.5).
    int16_t minSpeedCmS = (int16_t)lround(FOLLOW_MIN_COURSE_SPEED * 100.0);

    if (peer->gps.groundSpeed >= minSpeedCmS)
    {
        haveValidCourse = true;
        lastValidCourseDeg = (double)peer->gps.groundCourse / 10.0;
        return lastValidCourseDeg;
    }

    switch (FOLLOW_STATIONARY_MODE)
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

bool FollowManager::targetSane(const FollowOffset &offset, const FollowTarget &target)
{
    double horizontalMag = sqrt(offset.longitudinal_m * offset.longitudinal_m +
                                 offset.lateral_m * offset.lateral_m);
    double mag3d = sqrt(horizontalMag * horizontalMag + offset.vertical_m * offset.vertical_m);

    // Minimum 3D separation — forbids the degenerate collision slot (spec §7.4).
    if (mag3d < FOLLOW_MIN_SEP_M)
    {
        return false;
    }

    // Minimum vertical gap for stacked (overhead/underneath) slots — absorbs
    // GPS vertical error, not just physical clearance (spec §7.4).
    if (horizontalMag < FOLLOW_STACKED_HORIZONTAL_EPSILON_M && fabs(offset.vertical_m) < FOLLOW_MIN_VSEP_M)
    {
        return false;
    }

    // Runtime sanity: the solved target shouldn't be unreasonably far from
    // the follower's own position (spec §7.4).
    GNSSLocation targetLoc{};
    targetLoc.lat = (double)target.lat_1e7 / 1e7;
    targetLoc.lon = (double)target.lon_1e7 / 1e7;
    double distFromSelf = GNSSManager::getSingleton()->horizontalDistanceTo(targetLoc);
    if (distFromSelf > FOLLOW_MAX_TARGET_DIST_M)
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
    nextRunTime = millis() + (1000 / FOLLOW_EMIT_HZ);

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
