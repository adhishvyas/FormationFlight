#pragma once

#include "FollowManager.h"

// Free functions private to FollowManager.cpp's implementation (spec
// docs/spec/2026-09-03-FollowTestSuite.md §2.2/§4.14 calls them "genuinely
// pure... need no seam at all"). That's true of their logic, but not quite
// of their linkage: they were `static` (internal linkage), so a separate
// test translation unit couldn't call them at all. Promoted from `static`
// to ordinary external linkage and declared here purely so the native test
// suite can call them directly -- FollowManager.cpp is still the only
// production caller, and this is not part of FollowManager's public API.

bool offsetGeometrySane(const FollowOffset &offset, double minSepM, double minVSepM, String *errMsg);

bool axisSignLocked(double candidateAxis, double referenceAxis,
                     double candidateOther1, double candidateOther2,
                     double referenceOther1, double referenceOther2,
                     double minSepM);

bool candidateOffsetOk(const FollowOffset &candidate, const FollowOffset &reference,
                        double minSepM, double minVSepM);

bool rcCandidateMatchesStaticDefault(const FollowOffset &candidate, const FollowRuntimeConfig &config);
