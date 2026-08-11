#pragma once

// Compile-time configuration for FollowManager (Phase 1 — no runtime/EEPROM
// config yet, see docs/plans/2026-07-31-FollowMeOnInav-Plan.md). Every value
// here is #ifndef-guarded so a target's build_flags can override it without
// editing this file.
//
// All geometry/timing values that get combined with GPS-derived doubles
// (lat/lon in degrees, altitude in cm) are declared here as `double`, not
// `float`, so no precision is thrown away converting into/out of those
// calculations — see FollowManager.cpp for where these get combined.

enum FollowLongSlot {
    FOLLOW_LONG_AHEAD = 0,
    FOLLOW_LONG_CENTER = 1,
    FOLLOW_LONG_BEHIND = 2,
};

enum FollowLatSlot {
    FOLLOW_LAT_LEFT = 0,
    FOLLOW_LAT_CENTER = 1,
    FOLLOW_LAT_RIGHT = 2,
};

enum FollowVertSlot {
    FOLLOW_VERT_BELOW = 0,
    FOLLOW_VERT_LEVEL = 1,
    FOLLOW_VERT_ABOVE = 2,
};

enum FollowTriggerMode {
    FOLLOW_TRIGGER_GCSNAV = 0,
    FOLLOW_TRIGGER_AUX = 1,
};

enum FollowStationaryMode {
    FOLLOW_STATIONARY_HOLD_COURSE = 0,
    FOLLOW_STATIONARY_WORLD_FRAME = 1,
};

// Named-preset default: chase-high (behind, centered, above). Testable with
// the leader stationary on the ground, unlike a purely horizontal preset
// (spec §7.3).
#ifndef FOLLOW_SLOT_LONG
#define FOLLOW_SLOT_LONG FOLLOW_LONG_BEHIND
#endif
#ifndef FOLLOW_SLOT_LAT
#define FOLLOW_SLOT_LAT FOLLOW_LAT_CENTER
#endif
#ifndef FOLLOW_SLOT_VERT
#define FOLLOW_SLOT_VERT FOLLOW_VERT_ABOVE
#endif

#ifndef FOLLOW_GAP_LONG_M
#define FOLLOW_GAP_LONG_M 15.0
#endif
#ifndef FOLLOW_GAP_LAT_M
#define FOLLOW_GAP_LAT_M 15.0
#endif
#ifndef FOLLOW_GAP_VERT_M
#define FOLLOW_GAP_VERT_M 10.0
#endif

// Below this 3D slot magnitude, refuse to arm (spec §7.4).
#ifndef FOLLOW_MIN_SEP_M
#define FOLLOW_MIN_SEP_M 8.0
#endif
// Minimum |vertical| when both horizontal components are ~0 (stacked slot).
// 5m physical/collision clearance + 8m worst-case GPS vertical error budget
// (spec §7.4, plan's "Unit and altitude-frame correctness" section).
#ifndef FOLLOW_MIN_VSEP_M
#define FOLLOW_MIN_VSEP_M 13.0
#endif
// Runtime sanity bound on solved target distance from the follower (spec §7.4).
#ifndef FOLLOW_MAX_TARGET_DIST_M
#define FOLLOW_MAX_TARGET_DIST_M 50.0
#endif

// Absolute floor on the final commanded home-relative altitude, meters
// (spec §7.6). Distinct from FOLLOW_MIN_VSEP_M above: that governs the
// configured slot's separation from the leader; this clamps alt_cm itself
// so a low/descending/landing leader (or a BELOW slot) can never command
// the follower at or below this altitude. Applied as a clamp, not a
// rejection — converted to centimeters at the comparison site.
#ifndef FOLLOW_MIN_ALT_M
#define FOLLOW_MIN_ALT_M 3.0
#endif

#ifndef FOLLOW_TRIGGER_MODE
#define FOLLOW_TRIGGER_MODE FOLLOW_TRIGGER_GCSNAV
#endif

// 0 = FIRST_ACTIVE (lock onto the first non-lost peer with id > 0 at
// acquire time); nonzero = pin acquisition to that specific peer id (spec §6.3).
#ifndef FOLLOW_TARGET_PEER
#define FOLLOW_TARGET_PEER 0
#endif

#ifndef FOLLOW_EMIT_HZ
#define FOLLOW_EMIT_HZ 4
#endif

#ifndef FOLLOW_PEER_TIMEOUT_MS
#define FOLLOW_PEER_TIMEOUT_MS 1500
#endif

// m/s, human-facing — converted to cm/s at the comparison site against
// peer->gps.groundSpeed (spec §7.5, plan's unit-mismatch fix).
#ifndef FOLLOW_MIN_COURSE_SPEED
#define FOLLOW_MIN_COURSE_SPEED 2.0
#endif

#ifndef FOLLOW_STATIONARY_MODE
#define FOLLOW_STATIONARY_MODE FOLLOW_STATIONARY_HOLD_COURSE
#endif
