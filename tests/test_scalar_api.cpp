#include "doctest.h"
#include "scalar-profiler.hpp"

#include <cmath>
#include <vector>

namespace {

constexpr double kMaxVel = 1.5;
constexpr double kMaxAccel = 3.0;
constexpr double kDt = 0.01;

const ScalarProfileConfig kConfig{ kMaxVel, kMaxAccel, kDt };

} // namespace

TEST_CASE("generateScalarProfile produces samples one dt apart") {
    const auto samples = generateScalarProfile(2.0, kConfig);
    REQUIRE(samples.size() > 1);
    CHECK(samples.front().position == doctest::Approx(0.0));
    for (size_t i = 1; i < samples.size(); ++i) {
        CHECK(samples[i].time - samples[i - 1].time == doctest::Approx(kDt).epsilon(1e-4));
        CHECK(samples[i].position >= samples[i - 1].position - 1e-9);
    }
}

TEST_CASE("a 1D profile honours the velocity and acceleration limits") {
    const auto samples = generateScalarProfile(2.0, kConfig);
    REQUIRE(samples.size() > 1);

    const double maxDelta = kMaxAccel * kDt + 1e-6;
    for (size_t i = 0; i < samples.size(); ++i) {
        CHECK(samples[i].velocity <= kMaxVel + 1e-6);
        CHECK(samples[i].velocity >= -1e-6);
        if (i > 0) {
            CHECK(std::fabs(samples[i].velocity - samples[i - 1].velocity) <= maxDelta);
        }
    }
}

TEST_CASE("a 1D profile long enough to cruise reaches the top speed") {
    const auto samples = generateScalarProfile(3.0, kConfig);
    REQUIRE(!samples.empty());

    double peak = 0.0;
    for (const auto& s : samples) {
        peak = std::max(peak, s.velocity);
    }
    CHECK(peak == doctest::Approx(kMaxVel).epsilon(0.01));
}

TEST_CASE("a 1D profile lands exactly on the requested start and exit velocity") {
    for (double exitVel : {0.0, 0.4}) {
        CAPTURE(exitVel);
        const auto samples = generateScalarProfile(2.0, kConfig, 0.3, exitVel);
        REQUIRE(samples.size() > 1);
        CHECK(samples.front().velocity == doctest::Approx(0.3));
        CHECK(samples.back().velocity == doctest::Approx(exitVel));
    }
}

TEST_CASE("a 1D profile stops within the distance it is given") {
    const double distance = 1.0;
    const auto samples = generateScalarProfile(distance, kConfig, 0.0, 0.0);
    REQUIRE(!samples.empty());
    CHECK(samples.back().position == doctest::Approx(distance));

    // Stoppable everywhere: constant max braking from each sample must still
    // reach rest by the end.
    for (const auto& s : samples) {
        const double remaining = std::max(0.0, distance - s.position);
        CHECK(s.velocity <= std::sqrt(2.0 * kMaxAccel * remaining) + 1e-4);
    }
}

TEST_CASE("1D keyframes cap velocity at the requested distance and need no pre-sorting") {
    // Deliberately out of order: the API must sort them by distance.
    const std::vector<ScalarKeyframe> keyframes = {
        {kMaxVel, 2.0}, {0.3, 1.0}, {kMaxVel, 0.0}};

    const auto samples = generateScalarProfile(2.0, kConfig, 0.0, 0.0, keyframes);
    REQUIRE(!samples.empty());

    double velNearMid = -1.0;
    for (const auto& s : samples) {
        if (velNearMid < 0.0 && s.position >= 1.0) {
            velNearMid = s.velocity;
        }
    }
    REQUIRE(velNearMid >= 0.0);
    CHECK(velNearMid <= 0.3 + 0.05);
}

TEST_CASE("a non-positive distance yields no samples") {
    CHECK(generateScalarProfile(0.0, kConfig).empty());
    CHECK(generateScalarProfile(-1.0, kConfig).empty());
}

TEST_CASE("the profile is unit-agnostic: scaling the axis scales the profile") {
    // The same numbers read as radians instead of metres describe a turret
    // move, and the profiler treats them identically -- the shape depends only
    // on the ratios, not on what the unit means.
    const ScalarProfileConfig radians{ 3.0 /*rad/s*/, 6.0 /*rad/s^2*/, kDt };
    const auto turret = generateScalarProfile(4.0 /*rad*/, radians);
    const auto drive = generateScalarProfile(2.0, kConfig);

    // Both double every limit and the distance, so they take the same time and
    // the turret's velocity is exactly twice the drive's at each step.
    REQUIRE(turret.size() == drive.size());
    for (size_t i = 0; i < turret.size(); ++i) {
        CAPTURE(i);
        CHECK(turret[i].velocity == doctest::Approx(2.0 * drive[i].velocity).epsilon(1e-6));
    }
}
