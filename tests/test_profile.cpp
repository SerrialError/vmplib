#include "doctest.h"
#include "bezier.hpp"
#include "motion-profiling.hpp"
#include "ramsete.hpp"

#include <cmath>
#include <vector>

namespace {

constexpr float kMaxVel = 1.8885160604f;
constexpr float kMaxAccel = 4.12203073382f;
constexpr float kTrackWidth = 0.29508135f;
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
    return TrapezoidalProfile(pts, kMaxVel, kMaxAccel, kTrackWidth, 0.0f, startVel, endVel, {},
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

TEST_CASE("profile on a path shorter than the braking distance terminates") {
    // kShortPath is 0.30 m; braking from kMaxVel to rest needs 0.43 m, so the
    // braking limit is active from the very first step.
    TrapezoidalProfile profile = makeProfile(kShortPath, 0.f, 0.f);
    REQUIRE(runToCompletion(profile));
    CHECK(profile.getPoses().size() > 1);
}

TEST_CASE("profile brakes to the exit velocity by the end of the path") {
    // The final sample is taken up to one step short of the end, where the
    // braking ramp still permits v = sqrt(2*a*deltaS) with deltaS = v*dt.
    // Solving gives a floor of 2*a*dt, which bounds the overshoot.
    const float tolerance = 2.0f * kMaxAccel * kDt;

    for (float exitVel : {0.0f, 0.5f}) {
        CAPTURE(exitVel);
        TrapezoidalProfile profile = makeProfile(kLongPath, 0.f, exitVel);
        REQUIRE(runToCompletion(profile));
        CHECK(profile.getVelocities().back().linear <= exitVel + tolerance);
    }
}

TEST_CASE("braking limit keeps the profile stoppable at every point") {
    // At each sample the commanded speed must be low enough that constant
    // max deceleration still reaches exit velocity by the end of the path.
    TrapezoidalProfile profile = makeProfile(kLongPath, 0.f, 0.f);
    REQUIRE(runToCompletion(profile));

    const float total = sFunction(kLongPath, 1.0f);
    const auto& poses = profile.getPoses();
    const auto& vels = profile.getVelocities();

    float travelled = 0.f;
    for (size_t i = 1; i < poses.size(); ++i) {
        const float dx = poses[i].x - poses[i - 1].x;
        const float dy = poses[i].y - poses[i - 1].y;
        travelled += std::sqrt(dx * dx + dy * dy);

        const float remaining = std::max(0.f, total - travelled);
        const float stoppable = std::sqrt(2.0f * kMaxAccel * remaining);
        CHECK(vels[i].linear <= stoppable + 0.05f);
    }
}

// --- Keyframes -------------------------------------------------------------

TEST_CASE("keyframes cap velocity at the requested arc positions") {
    // Slow to 0.3 m/s halfway along the path, then release back to full speed.
    const std::vector<KeyframeVelocities> keyframes = {
        {kMaxVel, 0.0f}, {0.3f, 0.5f}, {kMaxVel, 1.0f}};

    TrapezoidalProfile profile(kLongPath, kMaxVel, kMaxAccel, kTrackWidth, 0.0f, 0.0f, 0.0f,
                               keyframes, true, kDt);
    REQUIRE(runToCompletion(profile));

    const float total = sFunction(kLongPath, 1.0f);
    const float sMid = sFunction(kLongPath, 0.5f);

    const auto& poses = profile.getPoses();
    const auto& vels = profile.getVelocities();

    float travelled = 0.f;
    float velNearMid = -1.f;
    for (size_t i = 1; i < poses.size(); ++i) {
        const float dx = poses[i].x - poses[i - 1].x;
        const float dy = poses[i].y - poses[i - 1].y;
        travelled += std::sqrt(dx * dx + dy * dy);
        if (velNearMid < 0.f && travelled >= sMid) {
            velNearMid = vels[i].linear;
        }
    }

    REQUIRE(total > 0.f);
    REQUIRE(velNearMid >= 0.f);
    // At the midpoint keyframe the cap is 0.3 m/s.
    CHECK(velNearMid <= 0.3f + 0.05f);
}

TEST_CASE("keyframe limit interpolates between the bracketing pair") {
    // Two intervals with different endpoints. If the interpolation measured
    // from the path start instead of from the preceding keyframe, the second
    // interval would be capped far too low.
    const std::vector<KeyframeVelocities> keyframes = {
        {0.2f, 0.0f}, {0.2f, 0.5f}, {kMaxVel, 1.0f}};

    TrapezoidalProfile profile(kLongPath, kMaxVel, kMaxAccel, kTrackWidth, 0.0f, 0.0f, 0.0f,
                               keyframes, true, kDt);
    REQUIRE(runToCompletion(profile));

    const auto& vels = profile.getVelocities();
    // The final interval ramps up to kMaxVel, so the profile must exceed the
    // 0.2 m/s plateau well before the end.
    const float peak = std::max_element(vels.begin(), vels.end(),
                                        [](const VelocityLayout& a, const VelocityLayout& b) {
                                            return a.linear < b.linear;
                                        })
                           ->linear;
    CHECK(peak > 0.4f);
}

TEST_CASE("keyframes are indexed by arc length, not elapsed time") {
    // A keyframe at t=0.9 must still be capping velocity after more than one
    // second of travel. The old code compared the parameter against
    // time_accum_, so every keyframe was skipped within the first second.
    const std::vector<KeyframeVelocities> keyframes = {
        {kMaxVel, 0.0f}, {kMaxVel, 0.85f}, {0.25f, 1.0f}};

    TrapezoidalProfile profile(kLongPath, kMaxVel, kMaxAccel, kTrackWidth, 0.0f, 0.0f, 0.25f,
                               keyframes, true, kDt);
    REQUIRE(runToCompletion(profile));

    const auto& vels = profile.getVelocities();
    REQUIRE(vels.back().time > 1.0f);
    CHECK(vels.back().linear <= 0.25f + 0.1f);
}

// --- Known-failing: documents bugs fixed in later PRs ----------------------

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

    RamseteFollower follower(profile.getPoses(), profile.getVelocities(), kTrackWidth, 2.0f,
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

TEST_CASE("ramsete converges from an initial pose offset") {
    TrapezoidalProfile profile = makeProfile(kLongPath, 0.f, 0.f);
    REQUIRE(runToCompletion(profile));

    const auto& refPoses = profile.getPoses();
    Pose offset = refPoses.front();
    offset.x += 0.10f;
    offset.y -= 0.10f;
    offset.theta += 0.25f;

    RamseteFollower follower(refPoses, profile.getVelocities(), kTrackWidth, 2.0f, 0.7f, 0.0f,
                             kDt, false, offset);

    const float initialError = std::sqrt(0.10f * 0.10f + 0.10f * 0.10f);

    std::vector<float> errors;
    size_t i = 0;
    int steps = 0;
    while (!follower.isFinished() && steps < kStepCap) {
        follower.step();
        const Pose actual = follower.getCurrentPose();
        const Pose ref = refPoses[i];
        errors.push_back(std::sqrt((actual.x - ref.x) * (actual.x - ref.x) +
                                   (actual.y - ref.y) * (actual.y - ref.y)));
        ++i;
        ++steps;
    }
    REQUIRE(errors.size() > 50);

    // The controller must actually pull the robot back onto the path.
    CHECK(errors.back() < initialError * 0.25f);

    // And it must not diverge on the way there.
    const float peak = *std::max_element(errors.begin(), errors.end());
    CHECK(peak <= initialError * 1.5f);
}
