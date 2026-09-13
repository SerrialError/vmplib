#include "doctest.h"
#include "config-error.hpp"
#include "velocity-profiler.hpp"

#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace {

constexpr double kMaxAccel = 800.0;   // rad/s^2
constexpr double kDt = 0.01;
constexpr double kStep = kMaxAccel * kDt;

const VelocityProfileConfig kConfig{ std::nullopt, kMaxAccel, kDt };

// Every velocity profile: one sample per dt from time 0, no step changing
// velocity by more than the acceleration limit allows, and each sample's accel
// the change it actually made.
void checkKinematics(const std::vector<VelocitySample>& samples) {
    REQUIRE(!samples.empty());
    CHECK(samples.front().time == 0.0);
    CHECK(samples.front().accel == 0.0);
    for (size_t i = 1; i < samples.size(); ++i) {
        CAPTURE(i);
        CHECK(samples[i].time == doctest::Approx(static_cast<double>(i) * kDt));
        CHECK(std::fabs(samples[i].velocity - samples[i - 1].velocity) <= kStep + 1e-9);
        CHECK(samples[i].accel ==
              doctest::Approx((samples[i].velocity - samples[i - 1].velocity) / kDt));
    }
}

} // namespace

TEST_CASE("a velocity ramp runs at the acceleration limit and lands exactly on the target") {
    const auto samples = generateVelocityProfile(kConfig, {{400.0}});
    checkKinematics(samples);
    // 400 rad/s at 8 rad/s per step is exactly 50 steps after the start sample.
    REQUIRE(samples.size() == 51);
    CHECK(samples.front().velocity == 0.0);
    CHECK(samples.back().velocity == 400.0);
    for (size_t i = 1; i < samples.size(); ++i) {
        CHECK(samples[i].accel == doctest::Approx(kMaxAccel));
    }
}

TEST_CASE("a target between whole steps is reached with a final partial step") {
    const auto samples = generateVelocityProfile(kConfig, {{403.0}});
    checkKinematics(samples);
    REQUIRE(samples.size() == 52);
    CHECK(samples.back().velocity == 403.0);
    CHECK(samples.back().accel == doctest::Approx(300.0));
}

TEST_CASE("a target is held for its hold time, rounded up to whole timesteps") {
    struct Case { double hold; size_t holdSamples; };
    // 0.07 / 0.01 computes as 7.000000000000001, which must still be 7 steps.
    for (const Case c : {Case{0.05, 5}, Case{0.055, 6}, Case{0.07, 7}, Case{0.0, 0}}) {
        CAPTURE(c.hold);
        const auto samples = generateVelocityProfile(kConfig, {{400.0, c.hold}});
        checkKinematics(samples);
        REQUIRE(samples.size() == 51 + c.holdSamples);
        for (size_t i = 51; i < samples.size(); ++i) {
            CHECK(samples[i].velocity == 400.0);
            CHECK(samples[i].accel == 0.0);
        }
    }
}

TEST_CASE("a reversal ramps through zero at the acceleration limit") {
    const auto samples = generateVelocityProfile(kConfig, {{400.0}, {-200.0}});
    checkKinematics(samples);
    CHECK(samples.back().velocity == -200.0);
    size_t zeros = 0;
    for (const auto& s : samples) {
        zeros += s.velocity == 0.0 ? 1 : 0;
    }
    // The start, and once on the way down.
    CHECK(zeros == 2);
}

TEST_CASE("a velocity profile can start already moving") {
    const auto samples = generateVelocityProfile(kConfig, {{100.0, 0.02}, {0.0}}, 100.0);
    checkKinematics(samples);
    CHECK(samples[1].velocity == 100.0);
    CHECK(samples[2].velocity == 100.0);
    CHECK(samples.back().velocity == 0.0);
}

TEST_CASE("maxVelocity is optional, and rejects faster targets when set") {
    const VelocityProfileConfig capped{ 450.0, kMaxAccel, kDt };
    CHECK_NOTHROW(generateVelocityProfile(capped, {{450.0}, {-450.0}}));
    CHECK_THROWS_AS(generateVelocityProfile(capped, {{500.0}}), VelocityTargetError);
    CHECK_THROWS_AS(generateVelocityProfile(capped, {{-500.0}}), VelocityTargetError);
    CHECK_THROWS_AS(generateVelocityProfile(capped, {{0.0}}, 500.0), VelocityTargetError);
    CHECK_NOTHROW(generateVelocityProfile(kConfig, {{5000.0}}));
}

TEST_CASE("a velocity profile rejects bad limits and targets") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    CHECK_THROWS_WITH_AS(generateVelocityProfile(VelocityProfileConfig{}, {{1.0}}),
                         "maxAccel is required but was not set", ConfigError);
    CHECK_THROWS_AS(generateVelocityProfile(VelocityProfileConfig{0.0, kMaxAccel}, {{1.0}}),
                    ConfigError);
    CHECK_THROWS_AS(generateVelocityProfile(VelocityProfileConfig{std::nullopt, kMaxAccel, 0.0},
                                            {{1.0}}),
                    ConfigError);

    CHECK_THROWS_AS(generateVelocityProfile(kConfig, {{nan}}), VelocityTargetError);
    CHECK_THROWS_AS(generateVelocityProfile(kConfig, {{inf}}), VelocityTargetError);
    CHECK_THROWS_AS(generateVelocityProfile(kConfig, {{1.0, -0.5}}), VelocityTargetError);
    CHECK_THROWS_AS(generateVelocityProfile(kConfig, {{1.0, inf}}), VelocityTargetError);
}

TEST_CASE("no velocity targets yield no samples") {
    CHECK(generateVelocityProfile(kConfig, {}).empty());
}
