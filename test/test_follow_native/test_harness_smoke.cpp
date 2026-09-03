// Phase 1 smoke test (spec docs/spec/2026-09-03-FollowTestSuite.md §6
// rollout plan) -- proves the native env + seams + fakes actually compile,
// link, and run together, independent of any specific §4 coverage.

#include <unity.h>

#include "FollowManager.h"
#include "FakeMsp.h"
#include "FakeGnss.h"
#include "FakePeers.h"

void test_follow_manager_constructs_with_fakes()
{
    FakeMsp msp;
    FakeGnss gnss;
    FakePeers peers;
    FollowManager fm(&msp, &gnss, &peers);

    DynamicJsonDocument doc(1024);
    fm.configJson(&doc);
    TEST_ASSERT_TRUE(doc.containsKey("ofsLongM"));
}
