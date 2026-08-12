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

// Runtime-editable mirror of every §9 key except FOLLOW_TRIGGER_MODE/
// FOLLOW_AUX_* (AUX trigger is Phase 2b, not implemented — trigger mode
// stays compile-time-only and is reported read-only in configJson()).
// RAM only (Phase 3A/plan): seeded from the compile-time #defines in
// FollowConfig.h at construction, mutated in place by WiFiManager's
// POST /followmanager/config (Phase 3B), lost on reboot until Phase 4
// adds EEPROM persistence.
struct FollowRuntimeConfig {
    // Canonical track-relative offset (spec §7.3), meters. The
    // AHEAD/BEHIND/LEFT/RIGHT/ABOVE/BELOW "friendly grid" some UIs present
    // is purely a client-side view over these signed values (html/follow.js)
    // — FollowManager itself only ever deals in this one representation.
    double ofsLongM = FOLLOW_OFS_LONG_M; // +ahead / -behind
    double ofsLatM = FOLLOW_OFS_LAT_M;   // +right / -left
    double ofsVertM = FOLLOW_OFS_VERT_M; // +above / -below

    // 0 = FIRST_ACTIVE, nonzero = pin to that peer id (spec §6.3).
    uint8_t targetPeer = FOLLOW_TARGET_PEER;
    uint16_t emitHz = FOLLOW_EMIT_HZ;
    uint32_t peerTimeoutMs = FOLLOW_PEER_TIMEOUT_MS;

    double minSepM = FOLLOW_MIN_SEP_M;
    double minVSepM = FOLLOW_MIN_VSEP_M;
    double maxTargetDistM = FOLLOW_MAX_TARGET_DIST_M;
    double minAltM = FOLLOW_MIN_ALT_M;

    double minCourseSpeed = FOLLOW_MIN_COURSE_SPEED;

    // Commanded nose heading (spec §7.7) — sent via WP#255's p1, not a
    // second MSP message. headingDeg's meaning depends on headingMode (see
    // FollowConfig.h's FollowHeadingMode).
    FollowHeadingMode headingMode = FOLLOW_HEADING_MODE;
    double headingDeg = FOLLOW_HEADING_DEG;
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
    // Resolved runtime config for GET /followmanager/config (Phase 3B),
    // spec §10.3.
    void configJson(JsonDocument *doc) const;
    const FollowRuntimeConfig &getConfig() const { return config; }
    // Validates newConfig (spec §7.4 geometry rules + basic field sanity).
    // On success, replaces the active config atomically and returns true;
    // if newConfig's targetPeer differs from the current one, forces a
    // fresh peer acquire (spec §6.3's "user changes a setting" escape
    // hatch). On failure, leaves the active config untouched, returns
    // false, and fills *errMsg.
    bool applyConfig(const FollowRuntimeConfig &newConfig, String *errMsg);
    static FollowManager *getSingleton();

private:
    FollowRuntimeConfig config;
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
    // Clears any held/locked peer and drops back to ACQUIRING so the next
    // loop() cycle re-scans under the (possibly just-changed) targetPeer
    // setting (spec §6.3's explicit escape hatch). Safe to call even when
    // the gate is inactive — loop() unconditionally forces IDLE in that
    // case regardless of what state this leaves behind.
    void forceReacquire();
    // Current config's canonical offset (spec §7.3), i.e. {ofsLongM,
    // ofsLatM, ofsVertM} as a FollowOffset.
    FollowOffset resolveOffset();
    // Leader's usable track course in plain degrees, applying the
    // low-speed/stationary fallback (spec §7.5).
    double resolveCourseDeg(const peer_t *peer);
    bool targetSane(const FollowOffset &offset, const FollowTarget &target);
    // Commanded nose heading for this cycle, resolved per config.headingMode
    // (spec §7.7). courseDeg is the already-computed resolveCourseDeg()
    // result, reused here instead of recomputed. Returns 0 for
    // FOLLOW_HEADING_OFF (the "don't touch heading" wire sentinel);
    // otherwise wraps into [1, 360] — a computed value of exactly 0 is
    // remapped to 360, since INAV's WP#255 handler treats p1 == 0 as "no
    // heading update," not due north.
    int16_t resolveHeadingDeg(const peer_t *peer, double courseDeg) const;
};
