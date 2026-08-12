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

// Which of the two editing surfaces (spec §7.3) currently governs
// resolveOffset(): the friendly grid (slot enums + gaps) or the advanced
// raw canonical meters. Explicit rather than inferred from "is anything
// nonzero" so a raw (0,0,0) offset is representable and unambiguous.
enum FollowOffsetMode {
    FOLLOW_OFFSET_MODE_GRID = 0,
    FOLLOW_OFFSET_MODE_RAW = 1,
};

// Runtime-editable mirror of every §9 key except FOLLOW_TRIGGER_MODE/
// FOLLOW_AUX_* (AUX trigger is Phase 2b, not implemented — trigger mode
// stays compile-time-only and is reported read-only in configJson()).
// RAM only (Phase 3A/plan): seeded from the compile-time #defines in
// FollowConfig.h at construction, mutated in place by WiFiManager's
// POST /followmanager/config (Phase 3B), lost on reboot until Phase 4
// adds EEPROM persistence.
struct FollowRuntimeConfig {
    FollowOffsetMode offsetMode = FOLLOW_OFFSET_MODE_GRID;

    FollowLongSlot slotLong = FOLLOW_SLOT_LONG;
    FollowLatSlot slotLat = FOLLOW_SLOT_LAT;
    FollowVertSlot slotVert = FOLLOW_SLOT_VERT;
    double gapLongM = FOLLOW_GAP_LONG_M;
    double gapLatM = FOLLOW_GAP_LAT_M;
    double gapVertM = FOLLOW_GAP_VERT_M;

    // Advanced/raw canonical offsets (spec §7.3) — only take effect when
    // offsetMode == FOLLOW_OFFSET_MODE_RAW.
    double ofsLongM = 0.0;
    double ofsLatM = 0.0;
    double ofsVertM = 0.0;

    // 0 = FIRST_ACTIVE, nonzero = pin to that peer id (spec §6.3).
    uint8_t targetPeer = FOLLOW_TARGET_PEER;
    uint16_t emitHz = FOLLOW_EMIT_HZ;
    uint32_t peerTimeoutMs = FOLLOW_PEER_TIMEOUT_MS;

    double minSepM = FOLLOW_MIN_SEP_M;
    double minVSepM = FOLLOW_MIN_VSEP_M;
    double maxTargetDistM = FOLLOW_MAX_TARGET_DIST_M;
    double minAltM = FOLLOW_MIN_ALT_M;

    double minCourseSpeed = FOLLOW_MIN_COURSE_SPEED;
    FollowStationaryMode stationaryMode = FOLLOW_STATIONARY_MODE;

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
    // Resolved runtime config for GET /followmanager/config (Phase 3B) —
    // reflects grid->canonical expansion regardless of offsetMode so the UI
    // can show both views (spec §10.3).
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
    // Expands cfg's grid (slot+gap) or returns its raw offset directly,
    // depending on cfg.offsetMode (spec §7.3). Shared by the control loop
    // (via resolveOffset()) and applyConfig()'s server-side §7.4 validation,
    // so both always agree on what a given config resolves to.
    FollowOffset offsetFromConfig(const FollowRuntimeConfig &cfg) const;
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
