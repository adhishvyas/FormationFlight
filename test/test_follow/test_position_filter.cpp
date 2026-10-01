// Controller-level tests for the leader/self position filters wired into
// FollowController by phases C2 (leader) and C3 (self) of
// docs/plans/2026-10-01-FollowPositionFiltering-Plan.md. Axis/primitive-level
// math is covered independently in test_position_filter_axis.cpp; these drive
// the real FollowController through FollowHarness, explicitly opting back
// into filtering (FollowHarness defaults it off -- see test_helpers.h).

#include <unity.h>

#include <cmath>
#include <vector>

#include "test_helpers.h"

// Deterministic, roughly zero-mean jitter in [-amplitude, amplitude] -- same
// generator as test_position_filter_axis.cpp's (duplicated rather than
// shared, matching this suite's existing per-file helper convention, e.g.
// follow.cpp/follow_filter.cpp each keep their own file-local deg1e7()).
static double pseudoNoise(uint32_t i, double amplitude) {
    uint32_t x = i * 2654435761u;
    x ^= x >> 13;
    x *= 2246822519u;
    x ^= x >> 16;
    const double frac = static_cast<double>(x % 10000u) / 10000.0;
    return (frac - 0.5) * 2.0 * amplitude;
}

// A random point within ~amplitudeM of (lat, lon): a random bearing and a
// distance in [0, amplitudeM), both derived from the noise generator above.
static void jitterPoint(double lat, double lon, uint32_t seed, double amplitudeM,
                        double* outLat, double* outLon) {
    const double distM = std::fabs(pseudoNoise(seed, amplitudeM));
    const double bearingDeg = pseudoNoise(seed + 500000u, 180.0) + 180.0;
    geo::pointAtDistance(lat, lon, distM, bearingDeg, *outLat, *outLon);
}

// ---- C2: leader filter ----

// spec SS9.1.1: materially less cycle-to-cycle jitter in the resolved target
// with filtering on, driving a smooth (slow, so the whole run stays well
// inside maxTargetDistM) leader track with injected position noise.
void test_leader_position_noise_rejection_reduces_target_jitter() {
    const double baseLat = 37.0;
    const double baseLon = -122.0;
    const double courseDeg = 90.0;  // east
    const double speedMps = 0.5;    // slow drift
    const double stepS = 0.3;       // matches FollowHarness::tick()'s 300ms step
    const double noiseAmplitudeM = 2.0;
    const int kSteps = 60;
    const int kWarmupSteps = 10;

    auto runScenario = [&](bool filterEnabled) {
        FollowHarness h;
        h.fc.gcsNav = true;
        FollowConfig cfg = configOf(h);
        cfg.positionFilterEnabled = filterEnabled;
        // Zero the chase offset (and the minSepM it would otherwise violate):
        // this isolates pure leader-position filtering from resolveCourseDeg()'s
        // own (separately tested) filtered-course jitter, which would
        // otherwise also perturb the target via slotToLatLon()'s offset
        // rotation and confound the comparison below.
        cfg.ofsLongM = 0.0;
        cfg.ofsLatM = 0.0;
        cfg.minSepM = 0.0;
        cfg.minVSepM = 0.0;
        const char* err = nullptr;
        TEST_ASSERT_TRUE(h.apply(cfg, &err));
        h.self.set(baseLat, baseLon);

        double lat = baseLat;
        double lon = baseLon;
        std::vector<int32_t> targetLats;
        std::vector<int32_t> targetLons;

        for (int i = 0; i < kSteps; i++) {
            double nextLat, nextLon;
            geo::pointAtDistance(lat, lon, speedMps * stepS, courseDeg, nextLat, nextLon);
            lat = nextLat;
            lon = nextLon;

            double noisyLat, noisyLon;
            jitterPoint(lat, lon, static_cast<uint32_t>(i), noiseAmplitudeM, &noisyLat, &noisyLon);

            h.setPeer(/*uid=*/1, noisyLat, noisyLon, /*speedMs=*/10.0, courseDeg);
            h.tick();

            const FollowStatus s = h.status();
            TEST_ASSERT_TRUE_MESSAGE(s.haveLastTarget, "target suppressed mid-run");
            targetLats.push_back(s.lastTarget.lat_1e7);
            targetLons.push_back(s.lastTarget.lon_1e7);
        }

        double sumSq = 0.0;
        int counted = 0;
        for (int i = kWarmupSteps + 1; i < kSteps; i++) {
            const double dLat = static_cast<double>(targetLats[i] - targetLats[i - 1]);
            const double dLon = static_cast<double>(targetLons[i] - targetLons[i - 1]);
            sumSq += dLat * dLat + dLon * dLon;
            counted++;
        }
        return sumSq / counted;
    };

    const double filteredVariance = runScenario(/*filterEnabled=*/true);
    const double rawVariance = runScenario(/*filterEnabled=*/false);

    TEST_ASSERT_TRUE_MESSAGE(filteredVariance < rawVariance * 0.6,
                             "filtered target should be materially smoother cycle-to-cycle");
}

// spec SS9.1.4: re-locking onto a different peer must not blend the new
// leader's position with the old one's trail.
void test_leader_filter_resets_on_relock_not_blended_with_previous_peer() {
    FollowHarness h;
    h.fc.gcsNav = true;
    FollowConfig cfg = configOf(h);
    cfg.positionFilterEnabled = true;
    cfg.targetUid = 1;  // pin to peer A first
    // This test is about the leader filter's reset, not the follower's own
    // position or maxTargetDistM's gate -- self is set once and never moved,
    // so widen the gate rather than teleport self to keep up with the
    // leader swap (the self filter has no reset of its own -- spec SS3.4 --
    // so a sudden multi-km jump in self's own reported fix would just lag,
    // tripping targetTooFar() and masking the very thing this test checks).
    cfg.maxTargetDistM = 1.0e6;
    const char* err = nullptr;
    TEST_ASSERT_TRUE(h.apply(cfg, &err));

    // Lock onto peer A and let several ticks converge the filter around it.
    const double aLat = 37.0;
    const double aLon = -122.0;
    h.self.set(aLat, aLon);
    for (int i = 0; i < 10; i++) {
        h.setPeer(/*uid=*/1, aLat, aLon, /*speedMs=*/10.0, /*courseDeg=*/0.0);
        h.tick();
    }
    TEST_ASSERT_EQUAL(FOLLOW_LOCK_LOCKED, h.status().state);
    TEST_ASSERT_EQUAL_UINT32(1u, h.status().lockedUid);

    // Peer B appears far away (~140km); pin targetUid to it -- forces an
    // immediate reacquire and, per spec SS3.2, a leaderFilter_ reset.
    const double bLat = 38.0;
    const double bLon = -121.0;
    h.setPeer(/*uid=*/2, bLat, bLon, /*speedMs=*/10.0, /*courseDeg=*/0.0);

    FollowConfig cfg2 = configOf(h);
    cfg2.targetUid = 2;
    TEST_ASSERT_TRUE(h.apply(cfg2, &err));
    TEST_ASSERT_EQUAL(FOLLOW_LOCK_ACQUIRING, h.status().state);  // forceReacquire() fired

    h.tick();  // re-locks onto B and captures its first leader-filter sample
    const FollowStatus s = h.status();
    TEST_ASSERT_EQUAL(FOLLOW_LOCK_LOCKED, s.state);
    TEST_ASSERT_EQUAL_UINT32(2u, s.lockedUid);
    TEST_ASSERT_TRUE(s.haveLastTarget);

    // The resolved target must reflect B's raw first sample, not a blend
    // with A's trail -- i.e. close to B, nowhere near the ~140km-distant A.
    const double targetLat = static_cast<double>(s.lastTarget.lat_1e7) / 1e7;
    const double targetLon = static_cast<double>(s.lastTarget.lon_1e7) / 1e7;
    const double distFromBM = geo::distanceM(bLat, bLon, targetLat, targetLon);
    TEST_ASSERT_TRUE_MESSAGE(distFromBM < 20.0, "target should sit near B, not a blend with A");
}

// spec SS3.3/SS9.1.6: the minCourseSpeed hold-last-course fallback must keep
// working with a filtered course, including through the filter's own
// transient (the velocity estimate briefly swinging through zero and
// reversing sign) once the leader genuinely stops.
void test_filtered_course_minCourseSpeed_fallback_holds_last_course() {
    FollowHarness h;
    h.fc.gcsNav = true;
    h.fc.headingHold = true;
    FollowConfig cfg = configOf(h);
    cfg.positionFilterEnabled = true;
    cfg.headingMode = FOLLOW_HEADING_COURSE;
    const char* err = nullptr;
    TEST_ASSERT_TRUE(h.apply(cfg, &err));

    // Fly a straight line east for long enough that the leader filter's
    // velocity estimate converges to a stable ~90 deg course.
    double lat = 37.0;
    double lon = -122.0;
    const double speedMps = 10.0;
    const double stepS = 0.3;  // matches FollowHarness::tick()'s 300ms step
    for (int i = 0; i < 20; i++) {
        double nextLat, nextLon;
        geo::pointAtDistance(lat, lon, speedMps * stepS, /*bearing=*/90.0, nextLat, nextLon);
        lat = nextLat;
        lon = nextLon;
        h.setPeer(/*uid=*/1, lat, lon, speedMps, /*courseDeg=*/90.0);
        h.self.set(lat, lon);
        h.tick();
    }
    TEST_ASSERT_INT16_WITHIN(5, 90, h.status().lastTargetHeadingDeg);

    // Leader stops dead (position no longer advances, though it keeps
    // rebroadcasting) -- the filtered velocity estimate decays through zero
    // and overshoots negative before settling, which would reverse the raw
    // filtered course output. The held heading must stay at ~90 throughout.
    for (int i = 0; i < 15; i++) {
        h.setPeer(/*uid=*/1, lat, lon, /*speedMs=*/10.0, /*courseDeg=*/90.0);
        h.tick();
        TEST_ASSERT_INT16_WITHIN(5, 90, h.status().lastTargetHeadingDeg);
    }
}

// ---- C3: self filter ----

// spec SS3.4/SS9.1.1 (follower side): materially less cycle-to-cycle jitter
// in the along-track-error-driven autothrottle target speed when the
// follower's own fix is noisy, leader held still so the target itself is
// stable and only self-position noise is in play.
void test_self_position_noise_rejection_reduces_autothrottle_jitter() {
    const double leaderLat = 37.0;
    const double leaderLon = -122.0;
    const double noiseAmplitudeM = 2.0;
    const int kSteps = 60;
    const int kWarmupSteps = 10;

    auto runScenario = [&](bool filterEnabled) {
        FollowHarness h;
        h.fc.gcsNav = true;
        h.fc.platform = FcPlatform::Airplane;
        FollowConfig cfg = configOf(h);
        cfg.positionFilterEnabled = filterEnabled;
        cfg.speedCorrectionAccelCmS2 = 50;
        cfg.minTargetSpeedMps = 0.0;
        cfg.maxTargetSpeedMps = 30.0;  // wide clamp so this test's speeds pass through unclamped
        const char* err = nullptr;
        TEST_ASSERT_TRUE(h.apply(cfg, &err));

        std::vector<int32_t> speeds;
        for (int i = 0; i < kSteps; i++) {
            h.setPeer(/*uid=*/1, leaderLat, leaderLon, /*speedMs=*/10.0, /*courseDeg=*/0.0);
            double noisySelfLat, noisySelfLon;
            jitterPoint(leaderLat, leaderLon, static_cast<uint32_t>(i), noiseAmplitudeM,
                       &noisySelfLat, &noisySelfLon);
            h.self.set(noisySelfLat, noisySelfLon);
            h.tick();

            const FollowStatus s = h.status();
            TEST_ASSERT_TRUE_MESSAGE(s.haveLastTarget, "target suppressed mid-run");
            speeds.push_back(s.targetSpeedCmS);
        }

        double sumSq = 0.0;
        int counted = 0;
        for (int i = kWarmupSteps + 1; i < kSteps; i++) {
            const double d = static_cast<double>(speeds[i] - speeds[i - 1]);
            sumSq += d * d;
            counted++;
        }
        return sumSq / counted;
    };

    const double filteredVariance = runScenario(/*filterEnabled=*/true);
    const double rawVariance = runScenario(/*filterEnabled=*/false);

    TEST_ASSERT_TRUE_MESSAGE(filteredVariance < rawVariance * 0.6,
                             "filtered autothrottle target speed should be materially smoother");
}

// spec SS3.4/SS9.1.1: FOLLOW_HEADING_POINT_LEADER's bearing-to-leader output
// must likewise be smoother under self-position jitter alone (leader fixed).
void test_point_leader_heading_stability_under_self_position_jitter() {
    const double selfLat = 37.0;
    const double selfLon = -122.0;
    double leaderLat = 0.0;
    double leaderLon = 0.0;
    geo::pointAtDistance(selfLat, selfLon, 30.0, /*bearing=*/45.0, leaderLat, leaderLon);
    const double noiseAmplitudeM = 1.5;
    const int kSteps = 60;
    const int kWarmupSteps = 10;

    auto runScenario = [&](bool filterEnabled) {
        FollowHarness h;
        h.fc.gcsNav = true;
        h.fc.headingHold = true;
        FollowConfig cfg = configOf(h);
        cfg.positionFilterEnabled = filterEnabled;
        cfg.headingMode = FOLLOW_HEADING_POINT_LEADER;
        const char* err = nullptr;
        TEST_ASSERT_TRUE(h.apply(cfg, &err));

        std::vector<int16_t> headings;
        for (int i = 0; i < kSteps; i++) {
            h.setPeer(/*uid=*/1, leaderLat, leaderLon, /*speedMs=*/10.0, /*courseDeg=*/0.0);
            double noisySelfLat, noisySelfLon;
            jitterPoint(selfLat, selfLon, static_cast<uint32_t>(i), noiseAmplitudeM,
                       &noisySelfLat, &noisySelfLon);
            h.self.set(noisySelfLat, noisySelfLon);
            h.tick();

            const FollowStatus s = h.status();
            TEST_ASSERT_TRUE_MESSAGE(s.haveLastTarget, "target suppressed mid-run");
            headings.push_back(s.lastTargetHeadingDeg);
        }

        double sumSq = 0.0;
        int counted = 0;
        for (int i = kWarmupSteps + 1; i < kSteps; i++) {
            const double d = static_cast<double>(headings[i] - headings[i - 1]);
            sumSq += d * d;
            counted++;
        }
        return sumSq / counted;
    };

    const double filteredVariance = runScenario(/*filterEnabled=*/true);
    const double rawVariance = runScenario(/*filterEnabled=*/false);

    TEST_ASSERT_TRUE_MESSAGE(filteredVariance < rawVariance * 0.6,
                             "filtered POINT_LEADER heading should be materially smoother");
}
