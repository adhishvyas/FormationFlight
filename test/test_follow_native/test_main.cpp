// Single Unity entry point for the test_follow_native suite. PlatformIO
// compiles every .cpp in this directory into one binary, so setUp()/
// tearDown()/main() must be defined exactly once -- the other test_*.cpp
// files here define only plain (non-static) test functions, declared
// extern below and registered with RUN_TEST().

#include <unity.h>

void setUp() {}
void tearDown() {}

// test_harness_smoke.cpp
extern void test_follow_manager_constructs_with_fakes();

// test_slot_geometry.cpp (spec §4.2)
extern void test_ahead_at_course_zero_is_due_north();
extern void test_behind_at_course_zero_is_due_south();
extern void test_right_at_course_zero_is_due_east();
extern void test_left_at_course_zero_is_due_west();
extern void test_behind_at_course_90_is_due_west();
extern void test_behind_at_course_270_is_due_east();
extern void test_ahead_bearing_wraps_near_360();
extern void test_round_trip_ahead();
extern void test_round_trip_combined_offset_nontrivial_course();

// test_geometry_sane.cpp (spec §4.3)
extern void test_geometry_sane_at_exactly_minSepM_passes();
extern void test_geometry_sane_just_under_minSepM_fails();
extern void test_non_stacked_slot_ignores_minVSepM();
extern void test_stacked_slot_under_minVSepM_fails();
extern void test_stacked_slot_at_exactly_minVSepM_passes();
extern void test_applyConfig_rejects_geometrically_insane_static_offset();

// test_rc_safety_net.cpp (spec §4.8)
extern void test_layer1_rejects_candidate_failing_geometry_alone();
extern void test_layer2_rejects_sign_flip_when_other_axes_below_minSepM();
extern void test_layer2_allows_sign_flip_when_other_axes_above_minSepM();
extern void test_zero_on_candidate_side_never_counts_as_crossed();
extern void test_zero_on_reference_side_never_counts_as_crossed();

int main(int argc, char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_follow_manager_constructs_with_fakes);

    RUN_TEST(test_ahead_at_course_zero_is_due_north);
    RUN_TEST(test_behind_at_course_zero_is_due_south);
    RUN_TEST(test_right_at_course_zero_is_due_east);
    RUN_TEST(test_left_at_course_zero_is_due_west);
    RUN_TEST(test_behind_at_course_90_is_due_west);
    RUN_TEST(test_behind_at_course_270_is_due_east);
    RUN_TEST(test_ahead_bearing_wraps_near_360);
    RUN_TEST(test_round_trip_ahead);
    RUN_TEST(test_round_trip_combined_offset_nontrivial_course);

    RUN_TEST(test_geometry_sane_at_exactly_minSepM_passes);
    RUN_TEST(test_geometry_sane_just_under_minSepM_fails);
    RUN_TEST(test_non_stacked_slot_ignores_minVSepM);
    RUN_TEST(test_stacked_slot_under_minVSepM_fails);
    RUN_TEST(test_stacked_slot_at_exactly_minVSepM_passes);
    RUN_TEST(test_applyConfig_rejects_geometrically_insane_static_offset);

    RUN_TEST(test_layer1_rejects_candidate_failing_geometry_alone);
    RUN_TEST(test_layer2_rejects_sign_flip_when_other_axes_below_minSepM);
    RUN_TEST(test_layer2_allows_sign_flip_when_other_axes_above_minSepM);
    RUN_TEST(test_zero_on_candidate_side_never_counts_as_crossed);
    RUN_TEST(test_zero_on_reference_side_never_counts_as_crossed);

    return UNITY_END();
}
