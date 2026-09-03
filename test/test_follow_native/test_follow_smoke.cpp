// Phase 1 smoke test (spec docs/spec/2026-09-03-FollowTestSuite.md §6
// rollout plan) -- proves the native env + seams + fakes actually compile
// and link together. Real coverage (§4) lands in later phases; this only
// exists to make "the harness works" independently verifiable right now.

#include <unity.h>

#include "FollowManager.h"
#include "FakeMsp.h"
#include "FakeGnss.h"
#include "FakePeers.h"

void setUp() {}
void tearDown() {}

static void test_follow_manager_constructs_with_fakes()
{
    FakeMsp msp;
    FakeGnss gnss;
    FakePeers peers;
    FollowManager fm(&msp, &gnss, &peers);

    DynamicJsonDocument doc(1024);
    fm.configJson(&doc);
    TEST_ASSERT_TRUE(doc.containsKey("ofsLongM"));
}

static void test_slotToLatLon_pure_ahead_at_zero_course()
{
    // Ahead 10m at course 0 (due north leader) from (0,0) should land
    // north of the origin: positive lat, ~0 lon (spec §4.2).
    FollowTarget t = slotToLatLon(0, 0, 0.0, 10.0, 0.0);
    TEST_ASSERT_TRUE(t.lat_1e7 > 0);
    TEST_ASSERT_INT32_WITHIN(1000, 0, t.lon_1e7);
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_follow_manager_constructs_with_fakes);
    RUN_TEST(test_slotToLatLon_pure_ahead_at_zero_course);
    return UNITY_END();
}
