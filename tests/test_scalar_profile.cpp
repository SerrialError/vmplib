#include "doctest.h"
#include "scalar-profile.hpp"

#include <cmath>
#include <vector>

namespace {

constexpr double kMaxVel = 1.5;
constexpr double kMaxAccel = 3.0;
constexpr double kDt = 0.01;

// A distance grid of `nodes` points evenly spaced over [0, distance]. Enough
// resolution to resolve keyframes that fall between nodes.
std::vector<double> evenGrid(double distance, int nodes) {
    std::vector<double> s(nodes);
    for (int i = 0; i < nodes; ++i) {
        s[i] = distance * static_cast<double>(i) / (nodes - 1);
    }
    return s;
}

std::vector<double> flatCeiling(int nodes, double ceiling) {
    return std::vector<double>(nodes, ceiling);
}

// A flat-ceiling profile: the pure trapezoid/triangle case, no keyframes.
ScalarProfile makeProfile(double distance, double startVel, double endVel,
                          std::vector<ScalarKeyframe> keyframes = {}) {
    constexpr int kNodes = 256;
    return ScalarProfile(distance, evenGrid(distance, kNodes),
                         flatCeiling(kNodes, kMaxVel), kMaxAccel, startVel, endVel, kDt,
                         std::move(keyframes));
}

constexpr int kStepCap = 20000;

bool runToCompletion(ScalarProfile& profile) {
    profile.start();
    int steps = 0;
    while (!profile.isFinished() && steps < kStepCap) {
        profile.step();
        ++steps;
    }
    return profile.isFinished();
}

} // namespace

TEST_CASE("a scalar profile terminates and produces samples") {
    ScalarProfile profile = makeProfile(2.0, 0.0, 0.0);
    REQUIRE(runToCompletion(profile));
    CHECK(profile.samples().size() > 1);
}

TEST_CASE("scalar samples advance in position and never exceed the ceiling") {
    ScalarProfile profile = makeProfile(2.0, 0.0, 0.0);
    REQUIRE(runToCompletion(profile));

    const auto& samples = profile.samples();
    for (size_t i = 0; i < samples.size(); ++i) {
        CHECK(samples[i].velocity <= kMaxVel + 1e-6);
        CHECK(samples[i].velocity >= -1e-6);
        if (i > 0) {
            CHECK(samples[i].position >= samples[i - 1].position - 1e-9);
        }
    }
}

TEST_CASE("scalar timestamps advance by exactly dt") {
    ScalarProfile profile = makeProfile(2.0, 0.0, 0.0);
    REQUIRE(runToCompletion(profile));

    const auto& samples = profile.samples();
    for (size_t i = 1; i < samples.size(); ++i) {
        CHECK(samples[i].time - samples[i - 1].time == doctest::Approx(kDt).epsilon(1e-4));
    }
}

TEST_CASE("the scalar profile ends at exactly the exit velocity") {
    for (double exitVel : {0.0, 0.5}) {
        CAPTURE(exitVel);
        ScalarProfile profile = makeProfile(2.0, 0.0, exitVel);
        REQUIRE(runToCompletion(profile));
        CHECK(profile.samples().back().velocity == doctest::Approx(exitVel));
        CHECK(profile.samples().back().position == doctest::Approx(2.0));
    }
}

TEST_CASE("the scalar profile respects the acceleration limit every step") {
    // Sweep the shapes that reach the exit velocity differently: braking from
    // cruise, starting faster than a short path can shed, and a non-zero exit.
    struct Case { const char* name; double distance; double startVel; double exitVel; };
    const Case cases[] = {
        {"rest to rest", 2.0, 0.0, 0.0},
        {"rest to 0.5", 2.0, 0.0, 0.5},
        {"cruising to rest", 2.0, kMaxVel, 0.0},
        {"too short to brake in", 0.1, 0.0, 0.0},
    };

    const double maxDelta = kMaxAccel * kDt + 1e-6;
    for (const Case& c : cases) {
        CAPTURE(c.name);
        ScalarProfile profile = makeProfile(c.distance, c.startVel, c.exitVel);
        REQUIRE(runToCompletion(profile));

        const auto& samples = profile.samples();
        for (size_t i = 1; i < samples.size(); ++i) {
            CAPTURE(i);
            CHECK(std::fabs(samples[i].velocity - samples[i - 1].velocity) <= maxDelta);
            // The reported acceleration is the realised change in velocity.
            CHECK(samples[i].accel == doctest::Approx((samples[i].velocity -
                                                       samples[i - 1].velocity) / kDt));
        }
        CHECK(samples.back().velocity == doctest::Approx(c.exitVel));
    }
}

TEST_CASE("braking keeps the scalar profile stoppable at every point") {
    const double distance = 2.0;
    ScalarProfile profile = makeProfile(distance, 0.0, 0.0);
    REQUIRE(runToCompletion(profile));

    for (const auto& sample : profile.samples()) {
        const double remaining = std::max(0.0, distance - sample.position);
        // The continuous ramp is the ceiling; the profile rides the discrete one
        // below it, so no discretization slack is needed here.
        const double stoppable = std::sqrt(2.0 * kMaxAccel * remaining);
        CHECK(sample.velocity <= stoppable + 1e-4);
    }
}

TEST_CASE("scalar keyframes cap velocity at the requested distance") {
    // Slow to 0.3 at the midpoint, then release back to full speed.
    const double distance = 2.0;
    const std::vector<ScalarKeyframe> keyframes = {
        {kMaxVel, 0.0}, {0.3, 1.0}, {kMaxVel, 2.0}};

    ScalarProfile profile = makeProfile(distance, 0.0, 0.0, keyframes);
    REQUIRE(runToCompletion(profile));

    double velNearMid = -1.0;
    for (const auto& sample : profile.samples()) {
        if (velNearMid < 0.0 && sample.position >= 1.0) {
            velNearMid = sample.velocity;
        }
    }
    REQUIRE(velNearMid >= 0.0);
    CHECK(velNearMid <= 0.3 + 0.05);
}

TEST_CASE("a scalar profile reports how far the final step overshot the end") {
    ScalarProfile profile = makeProfile(2.0, 0.0, 0.0);
    REQUIRE(runToCompletion(profile));

    const auto& samples = profile.samples();
    REQUIRE(samples.size() > 1);
    // A step advances by v*dt at the speed held where it began.
    const double lastStep = samples[samples.size() - 2].velocity * kDt;
    CHECK(profile.overshootDistance() >= 0.0);
    CHECK(profile.overshootDistance() <= lastStep + 1e-9);
}

TEST_CASE("a carry-over longer than the path passes straight through it") {
    const double distance = 0.1;
    const double carry = distance + 0.05;
    ScalarProfile profile(distance, evenGrid(distance, 64), flatCeiling(64, kMaxVel),
                          kMaxAccel, 0.5, 0.0, kDt, {}, carry);
    CHECK(profile.isFinished());
    CHECK(profile.samples().empty());
    // The unspent remainder has to keep going, not vanish at the join.
    CHECK(profile.overshootDistance() == doctest::Approx(0.05).epsilon(1e-3));
}
