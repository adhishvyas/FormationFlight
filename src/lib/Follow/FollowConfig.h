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

enum FollowTriggerMode {
    FOLLOW_TRIGGER_GCSNAV = 0,
    FOLLOW_TRIGGER_AUX = 1,
};

// Commanded nose heading, sent via WP#255's p1 field (spec §7.7) — not a
// second MSP message. Applies uniformly to rotorcraft and fixed-wing
// followers; no craft-type branch (see spec §7.7 for why that's safe).
enum FollowHeadingMode {
    FOLLOW_HEADING_OFF = 0,             // don't touch heading (p1 = 0, pre-§7.7 behavior)
    FOLLOW_HEADING_COURSE = 1,          // direction of travel (leader's course)
    FOLLOW_HEADING_POINT_LEADER = 2,    // bearing toward the leader's live position
    FOLLOW_HEADING_FIXED = 3,           // FOLLOW_HEADING_DEG as an absolute compass heading
    FOLLOW_HEADING_COURSE_RELATIVE = 4, // FOLLOW_HEADING_DEG added as an offset from course
};

// Named-preset default: chase-high (behind, centered, above) expressed
// directly as canonical track-relative offsets. Testable with the leader
// stationary on the ground, unlike a purely horizontal preset (spec §7.3).
// The AHEAD/BEHIND/LEFT/RIGHT/ABOVE/BELOW grid is a UI-only view over these
// signed meters now (see html/follow.js) — FollowManager only ever sees the
// canonical offset.
#ifndef FOLLOW_OFS_LONG_M
#define FOLLOW_OFS_LONG_M -15.0
#endif
#ifndef FOLLOW_OFS_LAT_M
#define FOLLOW_OFS_LAT_M 0.0
#endif
#ifndef FOLLOW_OFS_VERT_M
#define FOLLOW_OFS_VERT_M 10.0
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

#ifndef FOLLOW_HEADING_MODE
#define FOLLOW_HEADING_MODE FOLLOW_HEADING_POINT_LEADER
#endif

// Degrees. Meaning depends on FOLLOW_HEADING_MODE: absolute compass heading
// for FIXED, offset added to the leader's live course for COURSE_RELATIVE
// (spec §7.7) — unused by the other modes.
#ifndef FOLLOW_HEADING_DEG
#define FOLLOW_HEADING_DEG 0.0
#endif

// GVAR index for the primary lock-state indicator (spec §3), or -1 to
// disable (default — zero MSP traffic, zero OSD dependency until a pilot
// opts in via the web UI). INAV supports indices 0-7.
#ifndef FOLLOW_STATUS_GVAR_INDEX
#define FOLLOW_STATUS_GVAR_INDEX -1
#endif

// GVAR index for the secondary condition-code indicator (spec §3.2), or -1
// to disable. Independent of FOLLOW_STATUS_GVAR_INDEX — a pilot can enable
// either, both, or neither. Deliberately generic: only the altitude-floor
// clamp (code 1) is implemented today, but the field/GVAR isn't scoped to
// that one condition — future non-exclusive conditions get code 2, 3, etc.
// on this same slot without another rename.
#ifndef FOLLOW_CONDITION_FLAGS_GVAR_INDEX
#define FOLLOW_CONDITION_FLAGS_GVAR_INDEX -1
#endif

// RC axis control (spec docs/spec/2026-08-15-FollowRcAxisControl.md §6),
// 1-based MSP_RC channel number per axis, or -1 to disable (default — zero
// MSP_RC polling until a pilot opts in via the web UI, same -1-disables-
// by-default pattern as the GVAR index fields above).
#ifndef FOLLOW_RC_LONG_CHANNEL
#define FOLLOW_RC_LONG_CHANNEL -1
#endif
#ifndef FOLLOW_RC_LAT_CHANNEL
#define FOLLOW_RC_LAT_CHANNEL -1
#endif
#ifndef FOLLOW_RC_VERT_CHANNEL
#define FOLLOW_RC_VERT_CHANNEL -1
#endif

// Debug GVAR output: RAM-only toggle (FollowRuntimeConfig::debug in
// FollowManager.h — deliberately absent from FollowEepromRecord, so it's
// always off again after a reboot, never persisted). When on, the commanded
// target's position relative to the follower's own current location (not
// absolute lat/lon — those are ~9-10 digit numbers that don't fit any INAV
// Custom OSD Element's numeric display, which tops out at 5 digits/±99999;
// see docs/user-guide-follow-mode.md §6.5) plus alt/heading are written to
// these four fixed GVAR indices every loop() cycle, for bench-testing in the
// goggles without dedicating a status/condition GVAR slot to it. Off by
// default.
#ifndef FOLLOW_DEBUG_ENABLED
#define FOLLOW_DEBUG_ENABLED false
#endif
#ifndef FOLLOW_DEBUG_NORTH_GVAR_INDEX
#define FOLLOW_DEBUG_NORTH_GVAR_INDEX 0
#endif
#ifndef FOLLOW_DEBUG_EAST_GVAR_INDEX
#define FOLLOW_DEBUG_EAST_GVAR_INDEX 1
#endif
#ifndef FOLLOW_DEBUG_ALT_GVAR_INDEX
#define FOLLOW_DEBUG_ALT_GVAR_INDEX 2
#endif
#ifndef FOLLOW_DEBUG_HEADING_GVAR_INDEX
#define FOLLOW_DEBUG_HEADING_GVAR_INDEX 3
#endif

// Speed autothrottle (spec docs/spec/2026-08-28-FollowSpeedAutothrottle.md).
// GVAR indices, -1 = disabled, same convention as the status/condition GVARs
// above. INAV supports indices 0-7.
#ifndef FOLLOW_TARGET_SPEED_GVAR_INDEX
#define FOLLOW_TARGET_SPEED_GVAR_INDEX -1
#endif
#ifndef FOLLOW_AUTOTHROTTLE_ENGAGE_GVAR_INDEX
#define FOLLOW_AUTOTHROTTLE_ENGAGE_GVAR_INDEX -1
#endif
// Pilot's autothrottle arm switch (spec §3.2), 1-based MSP_RC channel, or -1
// = unassigned (always armed).
#ifndef FOLLOW_AUTOTHROTTLE_ENABLE_RC_CHANNEL
#define FOLLOW_AUTOTHROTTLE_ENABLE_RC_CHANNEL -1
#endif
// Armed-range bounds (µs), a closed range rather than a single switch-high
// threshold so the same two fields can express a 2-way, 3-way, or 6-pos
// switch's specific "armed" detent(s) (spec §7 open question).
#ifndef FOLLOW_AUTOTHROTTLE_ENABLE_MIN_THRESHOLD_US
#define FOLLOW_AUTOTHROTTLE_ENABLE_MIN_THRESHOLD_US 1700
#endif
#ifndef FOLLOW_AUTOTHROTTLE_ENABLE_MAX_THRESHOLD_US
#define FOLLOW_AUTOTHROTTLE_ENABLE_MAX_THRESHOLD_US 2100
#endif
// 0 = feedforward-only (pure leader-speed mirror) until bench-tuned (spec §7).
#ifndef FOLLOW_SPEED_CORRECTION_KP
#define FOLLOW_SPEED_CORRECTION_KP 0
#endif
// m/s. This clamp floor is the feature's only stall-safety mechanism this
// iteration (spec §1.4/§3.5) — set it with real margin above the airframe's
// actual stall speed (roughly a third above stall is a reasonable starting
// point), since there is no dynamic sink-rate protection yet.
#ifndef FOLLOW_MIN_TARGET_SPEED_MPS
#define FOLLOW_MIN_TARGET_SPEED_MPS 5.0
#endif
#ifndef FOLLOW_MAX_TARGET_SPEED_MPS
#define FOLLOW_MAX_TARGET_SPEED_MPS 30.0
#endif
