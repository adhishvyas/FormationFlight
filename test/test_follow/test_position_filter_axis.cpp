// Pure math tests for follow_filter.h/.cpp -- no FollowHarness, no
// FollowController. Controller-level consumers (the leader/self filter
// instances wired into FollowController) are tested in test_position_filter.cpp
// once that wiring lands (plan phases C2/C3).

#include <unity.h>

#include <algorithm>
#include <cmath>

#include "follow_filter.h"
#include "geo.h"

using namespace ff;

// Deterministic, roughly zero-mean jitter in [-amplitude, amplitude] -- a
// cheap integer hash rather than <random>, so the sequence (and therefore
// every assertion below) is identical on every platform/compiler.
double pseudoNoise(uint32_t i, double amplitude) {
    uint32_t x = i * 2654435761u;
    x ^= x >> 13;
    x *= 2246822519u;
    x ^= x >> 16;
    const double frac = static_cast<double>(x % 10000u) / 10000.0;  // [0, 1)
    return (frac - 0.5) * 2.0 * amplitude;
}

// ---- First-sample / no-op-guard behavior ----

void test_axis1d_first_sample_initializes_without_residual_math() {
    FollowFilterAxis1D axis;
    TEST_ASSERT_FALSE(axis.initialized);

    updateAxis1D(&axis, /*measurement=*/5.0, /*nowMs=*/1000, /*alpha=*/0.5, /*beta=*/0.1667);

    TEST_ASSERT_TRUE(axis.initialized);
    TEST_ASSERT_EQUAL_DOUBLE(5.0, axis.position);
    TEST_ASSERT_EQUAL_DOUBLE(0.0, axis.velocity);
    TEST_ASSERT_EQUAL_UINT32(1000, axis.lastUpdateMs);
}

void test_axis1d_duplicate_timestamp_is_noop() {
    FollowFilterAxis1D axis;
    updateAxis1D(&axis, 5.0, 1000, 0.5, 0.1667);
    updateAxis1D(&axis, 100.0, 1000, 0.5, 0.1667);  // same nowMs -- dtS == 0

    TEST_ASSERT_EQUAL_DOUBLE(5.0, axis.position);
    TEST_ASSERT_EQUAL_DOUBLE(0.0, axis.velocity);
    TEST_ASSERT_EQUAL_UINT32(1000, axis.lastUpdateMs);
}

void test_axis1d_out_of_order_timestamp_is_noop() {
    FollowFilterAxis1D axis;
    updateAxis1D(&axis, 5.0, 1000, 0.5, 0.1667);
    updateAxis1D(&axis, 6.0, 1200, 0.5, 0.1667);
    const double positionAfterSecond = axis.position;
    const double velocityAfterSecond = axis.velocity;

    updateAxis1D(&axis, 999.0, 900, 0.5, 0.1667);  // nowMs < lastUpdateMs -- dtS < 0

    TEST_ASSERT_EQUAL_DOUBLE(positionAfterSecond, axis.position);
    TEST_ASSERT_EQUAL_DOUBLE(velocityAfterSecond, axis.velocity);
    TEST_ASSERT_EQUAL_UINT32(1200, axis.lastUpdateMs);
}

// ---- Noise rejection on a smooth constant-velocity signal ----

void test_axis1d_tracks_constant_velocity_signal_with_reduced_variance() {
    const auto gains = resolveFilterGains(50);
    const double alpha = gains.first;
    const double beta = gains.second;

    const double velocity = 5.0;    // m/s, ground truth
    const double stepMs = 200.0;    // 5 Hz sampling
    const double noiseAmplitude = 1.5;  // metres, ordinary GPS scatter
    const int kSteps = 200;
    const int kWarmupSteps = 20;  // let the velocity estimate converge first

    FollowFilterAxis1D axis;
    uint32_t nowMs = 0;
    double rawSqErrSum = 0.0;
    double filteredSqErrSum = 0.0;
    double filteredAbsErrSum = 0.0;
    int counted = 0;

    for (int i = 0; i < kSteps; i++) {
        nowMs = static_cast<uint32_t>(i * stepMs);
        const double truePos = velocity * (nowMs / 1000.0);
        const double measurement = truePos + pseudoNoise(static_cast<uint32_t>(i), noiseAmplitude);

        updateAxis1D(&axis, measurement, nowMs, alpha, beta);

        if (i >= kWarmupSteps) {
            const double rawErr = measurement - truePos;
            const double filteredErr = axis.position - truePos;
            rawSqErrSum += rawErr * rawErr;
            filteredSqErrSum += filteredErr * filteredErr;
            filteredAbsErrSum += std::fabs(filteredErr);
            counted++;
        }
    }

    const double rawVariance = rawSqErrSum / counted;
    const double filteredVariance = filteredSqErrSum / counted;
    const double filteredMeanAbsErr = filteredAbsErrSum / counted;

    // Materially lower variance, not just a marginal improvement.
    TEST_ASSERT_TRUE(filteredVariance < rawVariance * 0.5);
    // A lagged-but-correct estimate, not a flattened one: the filtered
    // output still has to actually track the true line, well inside the
    // raw noise amplitude.
    TEST_ASSERT_TRUE(filteredMeanAbsErr < noiseAmplitude * 0.5);
}

// ---- Real maneuver (step change) is tracked, not ignored ----

void test_axis1d_tracks_step_change_with_bounded_lag() {
    const auto gains = resolveFilterGains(50);
    const double alpha = gains.first;
    const double beta = gains.second;

    FollowFilterAxis1D axis;
    uint32_t nowMs = 0;
    const double stepMs = 200.0;

    // Converge on a stationary signal first (axis settles at 0, velocity 0).
    for (int i = 0; i < 20; i++) {
        nowMs = static_cast<uint32_t>(i * stepMs);
        updateAxis1D(&axis, 0.0, nowMs, alpha, beta);
    }
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.0, axis.position);

    // A real maneuver: the measured position jumps to a new constant value
    // and holds there (simulating a leader's sudden, sustained turn). An
    // alpha-beta filter's step response is a damped oscillation, not a
    // monotonic ramp (it legitimately overshoots before settling) -- so this
    // asserts the lag is bounded and the response converges, not that every
    // single sample's error shrinks.
    const double stepTarget = 20.0;
    const int kPostStepSteps = 25;
    double maxAbsErr = 0.0;
    double errAfter3Steps = 0.0;
    double finalErr = 0.0;

    for (int i = 0; i < kPostStepSteps; i++) {
        nowMs = static_cast<uint32_t>((20 + i) * stepMs);
        updateAxis1D(&axis, stepTarget, nowMs, alpha, beta);

        const double err = std::fabs(axis.position - stepTarget);
        maxAbsErr = std::max(maxAbsErr, err);
        if (i == 2) {
            errAfter3Steps = err;
        }
        if (i == kPostStepSteps - 1) {
            finalErr = err;
        }
    }

    // Not ignored: within 3 samples (0.6s) the filter has already closed most
    // of the gap to the new value.
    TEST_ASSERT_TRUE(errAfter3Steps < std::fabs(stepTarget) * 0.5);
    // Bounded: the response doesn't blow up past the size of the step itself.
    TEST_ASSERT_TRUE(maxAbsErr <= std::fabs(stepTarget));
    // Converges: settles tightly onto the new value well within a second.
    TEST_ASSERT_TRUE(finalErr < 0.1);
}

// ---- Gain derivation (Benedict-Bordner, spec SS5) ----

void test_resolve_filter_gains_strength_0_is_minimal_smoothing() {
    const auto gains = resolveFilterGains(0);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 1.0, gains.first);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 1.0, gains.second);
}

void test_resolve_filter_gains_strength_50_matches_spec_example() {
    const auto gains = resolveFilterGains(50);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.5, gains.first);
    TEST_ASSERT_DOUBLE_WITHIN(1e-3, 0.1667, gains.second);
}

void test_resolve_filter_gains_strength_100_floors_alpha() {
    const auto gains = resolveFilterGains(100);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, kFollowFilterMinAlpha, gains.first);
    const double expectedBeta =
        (kFollowFilterMinAlpha * kFollowFilterMinAlpha) / (2.0 - kFollowFilterMinAlpha);
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, expectedBeta, gains.second);
}

void test_resolve_filter_gains_alpha_is_monotonic_non_increasing() {
    double lastAlpha = resolveFilterGains(0).first;
    const uint8_t strengths[] = {10, 30, 50, 70, 90, 100};
    for (uint8_t strength : strengths) {
        const double alpha = resolveFilterGains(strength).first;
        TEST_ASSERT_TRUE(alpha <= lastAlpha);
        lastAlpha = alpha;
    }
}

// ---- filteredLocation()/filteredCourseDeg()/filteredSpeedMps() round trip ----

void test_filtered_location_course_and_speed_round_trip_straight_line() {
    // Lightest smoothing (strength 0, alpha=beta=1): with a noise-free,
    // constant-velocity input the filter's position/velocity state locks
    // onto the raw signal within two samples (see follow_filter.cpp's
    // algorithm -- residual collapses to zero once velocity is correct).
    const auto gains = resolveFilterGains(0);
    const double alpha = gains.first;
    const double beta = gains.second;

    const double originLat = 47.0;
    const double originLon = 8.0;
    const double courseDeg = 90.0;  // due east
    const double speedMps = 10.0;
    const double stepS = 1.0;
    const int kSteps = 5;
    const double altM = 123.0;

    FollowPositionFilter filter;
    double lat = originLat;
    double lon = originLon;
    uint32_t nowMs = 0;
    int32_t lastLat1e7 = 0;
    int32_t lastLon1e7 = 0;

    for (int i = 0; i < kSteps; i++) {
        double nextLat = 0.0;
        double nextLon = 0.0;
        geo::pointAtDistance(lat, lon, speedMps * stepS, courseDeg, nextLat, nextLon);
        lat = nextLat;
        lon = nextLon;
        nowMs = static_cast<uint32_t>((i + 1) * stepS * 1000.0);
        lastLat1e7 = static_cast<int32_t>(std::lround(lat * 1e7));
        lastLon1e7 = static_cast<int32_t>(std::lround(lon * 1e7));

        updateFilterPosition(&filter, lastLat1e7, lastLon1e7, altM, nowMs, alpha, beta);
    }

    const double course = filteredCourseDeg(filter);
    const double speed = filteredSpeedMps(filter);
    TEST_ASSERT_DOUBLE_WITHIN(1.0, courseDeg, course);
    TEST_ASSERT_DOUBLE_WITHIN(0.1, speedMps, speed);

    const FollowFilteredLocation loc = filteredLocation(filter);
    // Within ~1m (1e-5 deg of latitude) of the last fed-in sample.
    TEST_ASSERT_INT32_WITHIN(100, lastLat1e7, loc.lat_1e7);
    TEST_ASSERT_INT32_WITHIN(150, lastLon1e7, loc.lon_1e7);
    TEST_ASSERT_DOUBLE_WITHIN(0.01, altM, loc.altM);
}
