#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include "FollowConfig.h"
#include "../Peers/PeerManager.h"

enum FollowLockState {
    FOLLOW_LOCK_IDLE = 0,
    FOLLOW_LOCK_ACQUIRING = 1,
    FOLLOW_LOCK_LOCKED = 2,
    FOLLOW_LOCK_LOCKED_HOLDING = 3,
};

// Resolved 3D slot offset in the leader's track-relative frame, meters.
// Kept as double (not float) since it's combined with GPS-derived doubles
// and FC altitude centimeters in FollowManager.cpp's target math.
struct FollowOffset {
    double longitudinal_m; // +ahead / -behind
    double lateral_m;      // +right / -left
    double vertical_m;     // +above / -below
};

struct FollowTarget {
    int32_t lat_1e7;
    int32_t lon_1e7;
};

// Projects a leader position + track-relative offset to an absolute lat/lon
// (spec §7.2). peer_lat_1e6/peer_lon_1e6 are peer_t::gps's raw internal
// representation (degrees x 1e6, see PeerManager.h / spec §5[A] — NOT the
// x1e7 MSP wire format). course_deg is plain degrees, clockwise from north.
FollowTarget slotToLatLon(int32_t peer_lat_1e6, int32_t peer_lon_1e6, double course_deg,
                          double long_m, double lat_m);

class FollowManager
{
public:
    void loop();
    // Read-only snapshot for GET /followmanager/status (spec §10.2): PeerLock
    // state, locked peer id/name (if any), gate active/inactive, and the last
    // computed target (if any target has been solved this session).
    void statusJson(JsonDocument *doc);
    static FollowManager *getSingleton();

private:
    FollowLockState state = FOLLOW_LOCK_IDLE;
    uint8_t lockedId = 0;
    char lockedName[NAME_LENGTH + 1] = "";
    double lastValidCourseDeg = 0.0;
    bool haveValidCourse = false;
    unsigned long nextRunTime = 0;

    // Last target actually emitted via sendFollowWaypoint() (i.e. it passed
    // targetSane()), for status reporting only — not used by the control loop.
    bool haveLastTarget = false;
    FollowTarget lastTarget{};
    int32_t lastTargetAltCm = 0;
    unsigned long lastTargetTime = 0;

    bool followSwitchActive();
    // Advances the PeerLock state machine (spec §6.3) and returns the peer to
    // track this cycle, or nullptr if we're still acquiring or holding.
    const peer_t *resolveLock();
    FollowOffset resolveOffset();
    // Leader's usable track course in plain degrees, applying the
    // low-speed/stationary fallback (spec §7.5).
    double resolveCourseDeg(const peer_t *peer);
    bool targetSane(const FollowOffset &offset, const FollowTarget &target);
};
