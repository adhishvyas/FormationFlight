#pragma once
//
// Recursive alpha-beta position/velocity filter feeding Follow's leader and
// follower position inputs. Split out of follow.h/.cpp as an independent,
// reusable math module -- the same way geo.cpp/h was already extracted --
// rather than inlined into FollowController: pure doubles, no Arduino/
// hardware deps, fully host-testable on its own.
//
// See docs/spec/2026-09-14-FollowPositionFiltering.md SS2-3 for the "why a
// recursive filter, not a windowed one" rationale and the exact algorithm
// this implements.
//
#include <cstdint>
#include <utility>

namespace ff {

// Floor on alpha so positionFilterStrengthPct=100 doesn't degenerate into a
// literal freeze (alpha=0 would mean the estimate never again moves after its
// first sample). See resolveFilterGains().
constexpr double kFollowFilterMinAlpha = 0.02;

// One recursive position+velocity channel. No history array -- see the file
// header for why this is recursive rather than windowed.
struct FollowFilterAxis1D {
    double position = 0.0;  // filtered estimate: metres in the local frame (north/east), or MSL metres for alt
    double velocity = 0.0;  // filtered estimate, m/s
    uint32_t lastUpdateMs = 0;
    bool initialized = false;
};

// One tracked point (the leader, or the follower itself), as three
// independent 1D channels sharing a flat-earth tangent-plane origin anchored
// at the first sample seen after a reset.
struct FollowPositionFilter {
    double originLat = 0.0;  // local tangent-plane anchor -- set once, on first sample
    double originLon = 0.0;
    bool haveOrigin = false;
    FollowFilterAxis1D north;  // metres north of origin
    FollowFilterAxis1D east;   // metres east of origin
    FollowFilterAxis1D alt;    // metres MSL
};

// Lat/lon (deg*1e7, the wire/MSP fixed-point format) + altitude (metres MSL),
// re-projected out of a FollowPositionFilter's north/east/alt state. A new
// small value type, not a reused FollowTarget/NodeLocation -- neither carries
// exactly this shape.
struct FollowFilteredLocation {
    int32_t lat_1e7 = 0;
    int32_t lon_1e7 = 0;
    double altM = 0.0;
};

// Per-axis predict/correct step. alpha/beta in (0, 1], from
// resolveFilterGains(). First call on a fresh axis (initialized == false)
// seeds position=measurement, velocity=0 and returns without running the
// residual math. A duplicate/out-of-order timestamp (dtS <= 0) is a no-op.
void updateAxis1D(FollowFilterAxis1D* axis, double measurement, uint32_t nowMs, double alpha,
                   double beta);

// Feeds one new position sample into all three channels of `filter`. Sets the
// filter's tangent-plane origin on the first sample seen since `filter` was
// last default-constructed/reset.
void updateFilterPosition(FollowPositionFilter* filter, int32_t lat_1e7, int32_t lon_1e7,
                           double altM, uint32_t nowMs, double alpha, double beta);

// Re-projects north/east through the origin back to lat/lon; alt passes
// through unchanged.
FollowFilteredLocation filteredLocation(const FollowPositionFilter& filter);

// atan2(east.velocity, north.velocity), wrapped to [0, 360).
double filteredCourseDeg(const FollowPositionFilter& filter);

// hypot(north.velocity, east.velocity).
double filteredSpeedMps(const FollowPositionFilter& filter);

// Benedict-Bordner critically-damped g-h filter gains derived from the single
// 0-100 "strength" knob (FollowConfig::positionFilterStrengthPct). 0 =
// lightest smoothing (alpha=1, beta=1), 100 = heaviest (alpha floored at
// kFollowFilterMinAlpha). Returns {alpha, beta}.
std::pair<double, double> resolveFilterGains(uint8_t strengthPct);

}  // namespace ff
