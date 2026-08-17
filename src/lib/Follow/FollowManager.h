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

// spec §5.3's condition-code table, sent via conditionFlagsGvarIndex.
enum FollowConditionCode {
    FOLLOW_CONDITION_NONE = 0,          // neither mechanism active
    FOLLOW_CONDITION_FLOOR_CLAMPED = 1, // altitude floor clamped, unrelated to RC
    FOLLOW_CONDITION_RC_INVALID_GAP_SETTINGS = 2,     // RC-attributable invalid gap rettings, and/or rcSlotFrozen (spec §5.2: never disagree)
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

    // GVAR indices (spec §3.4), -1 = disabled. Range enforced in
    // applyConfig() (-1 or 0-7); the web UI additionally makes an
    // out-of-range value structurally unreachable via a <select>.
    int16_t statusGvarIndex = FOLLOW_STATUS_GVAR_INDEX;
    int16_t conditionFlagsGvarIndex = FOLLOW_CONDITION_FLAGS_GVAR_INDEX;

    // RC axis control (spec §6): 1-based MSP_RC channel per axis, or -1 =
    // disabled. Range enforced in applyConfig() (-1 or 1-16). The configured
    // ofs{Long,Lat,Vert}M value becomes that axis's live-scaled bound once a
    // channel is assigned (spec §2.2), not a fixed point.
    int16_t rcLongChannel = FOLLOW_RC_LONG_CHANNEL;
    int16_t rcLatChannel = FOLLOW_RC_LAT_CHANNEL;
    int16_t rcVertChannel = FOLLOW_RC_VERT_CHANNEL;

    // RAM only, deliberately absent from FollowEepromRecord below — always
    // resets to false on reboot rather than persisting (see FollowConfig.h's
    // FOLLOW_DEBUG_ENABLED comment). When true, the follower's own commanded
    // waypoint lat/lon/alt/heading are written to GVARs 4-7 every loop() cycle.
    bool debug = FOLLOW_DEBUG_ENABLED;
};

// EEPROM persistence (Phase 4B): FollowRuntimeConfig's on-disk mirror, kept
// as its own versioned record so a fresh/uninitialized EEPROM region (or a
// future struct layout change) is detected independently of cfg's own
// VERSION_CONFIG (main.h) — a version mismatch here just means "nothing
// saved yet," not "corrupt," so FollowManager falls back to the
// compile-time defaults FollowRuntimeConfig's member initializers already
// seeded. Lives immediately after cfg's own EEPROM footprint — see
// ConfigHandler.cpp's config_init(), which sizes EEPROM.begin() to fit both.
//
// FollowRuntimeConfig's geometry/timing fields are `double` because
// FollowManager.cpp's target math combines them with GPS-derived doubles
// (spec §6.2/§7.2/§7.3) — but the web UI can never actually produce a
// fractional value for any of them: every type="number" field in
// html/follow.js goes through html/components.js's TextValue, which calls
// parseInt() on the input before it ever reaches state. So the EEPROM
// mirror stores those fields as int16_t instead of double — a quarter the
// size, with no precision loss for anything the UI can send — and
// FollowManager.cpp's toEepromRecord()/fromEepromRecord() do the
// int16_t<->double conversion in the one place it's needed. (int16_t's
// +-32767 range comfortably covers every field here: offsets/distances in
// meters, speed in m/s, heading in degrees.) targetPeer/emitHz/
// peerTimeoutMs/headingMode are already integer types in
// FollowRuntimeConfig, so they're carried through unchanged, no conversion
// needed.
#define FOLLOW_EEPROM_VERSION 4
struct FollowEepromRecord {
    uint16_t version;

    int16_t ofsLongM;
    int16_t ofsLatM;
    int16_t ofsVertM;

    uint8_t targetPeer;
    uint16_t emitHz;
    uint32_t peerTimeoutMs;

    int16_t minSepM;
    int16_t minVSepM;
    int16_t maxTargetDistM;
    int16_t minAltM;

    int16_t minCourseSpeed;

    FollowHeadingMode headingMode;
    int16_t headingDeg;

    int16_t statusGvarIndex;
    int16_t conditionFlagsGvarIndex;

    int16_t rcLongChannel;
    int16_t rcLatChannel;
    int16_t rcVertChannel;
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
    // Loads the persisted config from EEPROM if present and valid (Phase
    // 4B). Called once at startup, after ConfigHandler's config_init() has
    // already called EEPROM.begin(). Leaves the compile-time-seeded config
    // untouched if there's nothing valid to load (fresh flash, version
    // mismatch, or a record that fails applyConfig()'s validation).
    void loadFromEEPROM();
    // Persists the current in-memory config to its EEPROM region (Phase
    // 4B) — an explicit action distinct from applyConfig()'s RAM-only
    // mutation. Rate-limited so rapid repeated calls (e.g. accidental
    // double-clicks) don't hammer EEPROM; on failure (including
    // rate-limiting) returns false and fills *errMsg.
    bool saveToEEPROM(String *errMsg);
    static FollowManager *getSingleton();

private:
    FollowRuntimeConfig config;
    unsigned long lastEepromCommitMs = 0;
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

    // Last value actually written to each GVAR (spec §3.3's change+heartbeat
    // send rule), or INT32_MIN as a "never sent yet" sentinel so the very
    // first loop() cycle always sends — this is what satisfies spec §3.1's
    // "write 0 explicitly at startup," with no separate startup-only code path.
    int32_t lastSentStatusGvarValue = INT32_MIN;
    int32_t lastSentConditionFlagsGvarValue = INT32_MIN;
    unsigned long lastStatusGvarSendMs = 0;
    unsigned long lastConditionFlagsGvarSendMs = 0;

    // "Last known good" RC-scaled offset triple (spec §4.4) — the freeze
    // target when a candidate fails either safety layer. Bootstrapped to the
    // compile-time static offset, matching FollowRuntimeConfig's own member
    // initializers; applyConfig() resets this to the new config's static
    // offset on every successful apply (spec §4.4's reset rule).
    FollowOffset lastKnownGood{FOLLOW_OFS_LONG_M, FOLLOW_OFS_LAT_M, FOLLOW_OFS_VERT_M};
    // Whether resolveOffset()'s last cycle held lastKnownGood frozen rather
    // than adopting a fresh candidate (spec §4.4/§4.5), for statusJson()/
    // updateStatusGvars().
    bool rcSlotFrozen = false;
    // §4.6's ground-only advisory result, recomputed every loop() cycle while
    // disarmed and reset to false every cycle otherwise (spec §7's gating).
    bool rcPreArmCheckFailed = false;
    // The RC-scaled candidate §4.6's check evaluated this cycle, kept around
    // so statusJson() can report actual numbers (not just pass/fail) for
    // bench-testing RC scaling before arming. Same gating as
    // rcPreArmCheckFailed — havePreArmCandidateOffset false whenever the
    // craft isn't disarmed-with-an-axis-assigned, so a stale value never
    // lingers into a cycle where it wasn't recomputed.
    FollowOffset preArmCandidateOffset{};
    bool havePreArmCandidateOffset = false;
    // Offset triple actually used for the last emitted waypoint (spec §7
    // liveOffset), distinct from lastKnownGood which persists even on a
    // targetSane() rejection where no waypoint went out.
    FollowOffset lastLiveOffset{};

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
    // ofsLatM, ofsVertM} as a FollowOffset, scaled live by any RC-assigned
    // axis and passed through the two-layer geometry safety net (spec §3/§4).
    FollowOffset resolveOffset();
    // Per-axis RC-to-offset mapping (spec §3.1/§3.2). channel1Based < 1 means
    // "no channel assigned" -> returns configuredM unchanged (the only
    // fallback case). Once a channel is assigned, whatever value comes back
    // is mapped unconditionally, including a raw reading below 1000us -- it
    // just clamps to the 1000 endpoint like any other out-of-range value
    // (spec §3.1/§3.2). getRcChannelUs() returning false (no FC connected, or
    // an out-of-MSP_RC-range channel number) is the one case where there's
    // genuinely no value to map, so that also falls back to configuredM.
    double resolveAxisOffset(double configuredM, int16_t channel1Based) const;
    // {resolveAxisOffset(config.ofs{Long,Lat,Vert}M, config.rc{Long,Lat,Vert}Channel)}
    // as a triple — shared by resolveOffset() and the §4.6 pre-arm check so
    // both build the exact same candidate from the same inputs.
    FollowOffset resolveCandidateOffset() const;
    bool anyRcChannelAssigned() const;
    // Leader's usable track course in plain degrees, applying the
    // low-speed/stationary fallback (spec §7.5).
    double resolveCourseDeg(const peer_t *peer);
    bool targetSane(const FollowOffset &offset, const FollowTarget &target);
    // Commanded nose heading for this cycle, resolved per config.headingMode
    // (spec §7.7). courseDeg is the already-computed resolveCourseDeg()
    // result, reused here instead of recomputed. Returns 0 for
    // FOLLOW_HEADING_OFF (the "don't touch heading" wire sentinel);
    // otherwise wraps into [1, 359] — INAV's WP#255 handler only applies a
    // heading update when 0 < p1 < 360 (both ends exclusive), so a computed
    // value of exactly 0/360 is remapped to 1, not 360.
    int16_t resolveHeadingDeg(const peer_t *peer, double courseDeg) const;
    // Derives this cycle's GVAR values from current state and sends whichever
    // of the two configured GVARs (spec §3.4) changed or are due for their
    // heartbeat resend (spec §3.3). conditionCode is the caller-computed
    // spec §5.3 0/1/2 value for conditionFlagsGvarIndex — callers combine the
    // altitude-floor clamp and RC-freeze conditions (spec §5.1/§5.2) before
    // calling this, so this function no longer derives it itself.
    void updateStatusGvars(FollowConditionCode conditionCode);
    // Writes the follower's just-computed commanded waypoint (lat/lon/alt/
    // heading — the same values passed to sendFollowWaypoint()) to GVARs 4-7
    // (FOLLOW_DEBUG_*_GVAR_INDEX) every loop() cycle, gated on config.debug.
    void updateDebugGvars(int32_t lat_1e7, int32_t lon_1e7, int32_t altCm, int16_t headingDeg);
};
