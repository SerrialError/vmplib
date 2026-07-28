#include "doctest.h"
#include "bezier.hpp"
#include "motion-profiling.hpp"
#include "ramsete.hpp"

#include <cmath>
#include <vector>

namespace {

constexpr float kMaxVel = 1.8885160604f;
constexpr float kMaxAccel = 4.12203073382f;
constexpr float kDt = 0.01f;

// A gentle S-curve roughly 1.4 m long.
const std::vector<Point> kLongPath = {
    {0.f, 0.f}, {0.5f, 0.f}, {0.5f, 1.f}, {1.f, 1.f}};

// 0.30 m long: shorter than the 0.43 m needed to brake from kMaxVel to rest.
const std::vector<Point> kShortPath = {
    {0.f, 0.f}, {0.1f, 0.f}, {0.2f, 0.f}, {0.3f, 0.f}};

// A hairpin: curvature spikes hard in the middle of the segment.
const std::vector<Point> kHairpin = {
    {0.f, 0.f}, {1.2f, 0.f}, {1.2f, 0.25f}, {0.f, 0.25f}};

TrapezoidalProfile makeProfile(const std::vector<Point>& pts, float startVel, float endVel) {
    const float totalLength = sFunction(pts, 1.0f);
    const float decelDist =
        totalLength - (kMaxVel * kMaxVel - endVel * endVel) / (2.0f * kMaxAccel);

    return TrapezoidalProfile(pts, kMaxVel, kMaxAccel, decelDist, 0.0f, startVel, endVel, {},
                              false, kDt);
}

// Steps the profile with a hard cap so a non-terminating profile fails rather
// than hanging the test binary.
constexpr int kStepCap = 20000;

bool runToCompletion(TrapezoidalProfile& profile) {
    profile.start();
    int steps = 0;
    while (!profile.isFinished() && steps < kStepCap) {
        profile.step();
        ++steps;
    }
    return profile.isFinished();
}

} // namespace

TEST_CASE("profile on a normal-length path terminates") {
    TrapezoidalProfile profile = makeProfile(kLongPath, 0.f, 0.f);
    REQUIRE(runToCompletion(profile));
    CHECK(profile.getPoses().size() > 1);
    CHECK(profile.getVelocities().size() == profile.getPoses().size());
}

TEST_CASE("profile never exceeds the maximum linear velocity") {
    TrapezoidalProfile profile = makeProfile(kLongPath, 0.f, 0.f);
    REQUIRE(runToCompletion(profile));

    for (const auto& v : profile.getVelocities()) {
        CHECK(v.linear <= kMaxVel + 1e-4f);
        CHECK(v.linear >= -1e-4f);
    }
}

TEST_CASE("profile timestamps advance by exactly dt") {
    TrapezoidalProfile profile = makeProfile(kLongPath, 0.f, 0.f);
    REQUIRE(runToCompletion(profile));

    const auto& vels = profile.getVelocities();
    for (size_t i = 1; i < vels.size(); ++i) {
        CHECK(vels[i].time - vels[i - 1].time == doctest::Approx(kDt).epsilon(1e-4));
    }
}

TEST_CASE("profile ends at the end of the path") {
    TrapezoidalProfile profile = makeProfile(kLongPath, 0.f, 0.f);
    REQUIRE(runToCompletion(profile));

    const Pose last = profile.getPoses().back();
    CHECK(last.x == doctest::Approx(kLongPath.back().x).epsilon(1e-3));
    CHECK(last.y == doctest::Approx(kLongPath.back().y).epsilon(1e-3));
}

TEST_CASE("angular velocity is consistent with curvature and linear velocity") {
    TrapezoidalProfile profile = makeProfile(kLongPath, 0.f, 0.f);
    REQUIRE(runToCompletion(profile));

    // omega = kappa * v, so |omega| must stay bounded by the worst-case
    // curvature along the path times the commanded speed.
    float maxKappa = 0.f;
    for (int i = 0; i <= 100; ++i) {
        maxKappa = std::max(maxKappa, unsignedCurvature(kLongPath, i / 100.f));
    }
    for (const auto& v : profile.getVelocities()) {
        CHECK(std::fabs(v.angular) <= maxKappa * kMaxVel + 1e-3f);
    }
}

// --- Known-failing: documents bugs fixed in later PRs ----------------------

TEST_CASE("profile on a short path terminates" * doctest::should_fail()) {
    // M3: decelDist goes negative when the path is shorter than the braking
    // distance, so the acceleration branch never fires and the commanded speed
    // stays pinned at the start velocity (zero), advancing t by nothing.
    TrapezoidalProfile profile = makeProfile(kShortPath, 0.f, 0.f);
    CHECK(runToCompletion(profile));
}

TEST_CASE("commanded velocity respects the acceleration limit" * doctest::should_fail()) {
    // M5: the greedy planner caps how fast velocity may *rise*, but nothing
    // caps how fast it may fall, so a curvature spike produces a step change
    // that no real drivetrain can track. Fixed by the forward/backward pass.
    TrapezoidalProfile profile = makeProfile(kHairpin, 0.f, 0.f);
    REQUIRE(runToCompletion(profile));

    const auto& vels = profile.getVelocities();
    const float maxDelta = kMaxAccel * kDt;
    for (size_t i = 1; i < vels.size(); ++i) {
        CHECK(std::fabs(vels[i].linear - vels[i - 1].linear) <= maxDelta + 1e-4f);
    }
}

// --- RAMSETE ---------------------------------------------------------------

TEST_CASE("ramsete reproduces the reference when it starts on it") {
    TrapezoidalProfile profile = makeProfile(kLongPath, 0.f, 0.f);
    REQUIRE(runToCompletion(profile));

    RamseteFollower follower(profile.getPoses(), profile.getVelocities(), 0.29508135f, 2.0f,
                             0.7f, 0.0f, kDt, false);

    int steps = 0;
    while (!follower.isFinished() && steps < kStepCap) {
        follower.step();
        ++steps;
    }
    REQUIRE(follower.isFinished());
    CHECK(follower.getExecutedPoses().size() == profile.getPoses().size());

    // Starting exactly on the reference, tracking error must stay small.
    const Pose finalPose = follower.getExecutedPoses().back();
    const Pose refPose = profile.getPoses().back();
    CHECK(finalPose.x == doctest::Approx(refPose.x).epsilon(0.05));
    CHECK(finalPose.y == doctest::Approx(refPose.y).epsilon(0.05));
}
