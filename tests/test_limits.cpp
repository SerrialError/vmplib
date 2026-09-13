#include "doctest.h"
#include "config-error.hpp"
#include "motion-profiler.hpp"
#include "scalar-profiler.hpp"

#include <limits>
#include <optional>
#include <vector>

namespace {

const std::vector<std::vector<Point>> kStraight = {
    {{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}}};
const std::vector<std::vector<KeyframeVelocitiesXandY>> kNoKeyframes = {{}};

// Every value a limit must not take: zero and negative make no physical sense,
// and NaN or infinity poison every sum the profiler builds from them.
const double kInvalid[] = {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity()};

ProfileConfig validPathConfig() {
    ProfileConfig config;
    config.maxVelocity = 1.5;
    config.maxAccel = 3.0;
    config.trackWidth = 0.3;
    return config;
}

} // namespace

TEST_CASE("generateScalarProfile rejects a limit that is not positive and finite") {
    for (double bad : kInvalid) {
        CAPTURE(bad);
        CHECK_THROWS_AS(generateScalarProfile(1.0, ScalarProfileConfig{bad, 3.0, 0.01}),
                        ConfigError);
        CHECK_THROWS_AS(generateScalarProfile(1.0, ScalarProfileConfig{1.5, bad, 0.01}),
                        ConfigError);
        CHECK_THROWS_AS(generateScalarProfile(1.0, ScalarProfileConfig{1.5, 3.0, bad}),
                        ConfigError);
    }
}

TEST_CASE("generateScalarProfile accepts valid limits") {
    CHECK_NOTHROW(generateScalarProfile(1.0, ScalarProfileConfig{1.5, 3.0, 0.01}));
}

TEST_CASE("generateTrajectory rejects a limit that is not positive and finite") {
    for (double bad : kInvalid) {
        CAPTURE(bad);
        ProfileConfig config = validPathConfig();
        config.maxVelocity = bad;
        CHECK_THROWS_AS(generateTrajectory(kStraight, kNoKeyframes, false, config), ConfigError);

        config = validPathConfig();
        config.maxAccel = bad;
        CHECK_THROWS_AS(generateTrajectory(kStraight, kNoKeyframes, false, config), ConfigError);

        config = validPathConfig();
        config.trackWidth = bad;
        CHECK_THROWS_AS(generateTrajectory(kStraight, kNoKeyframes, false, config), ConfigError);

        config = validPathConfig();
        config.dt = bad;
        CHECK_THROWS_AS(generateTrajectory(kStraight, kNoKeyframes, false, config), ConfigError);
    }
}

TEST_CASE("a profile refuses to run with a robot limit left unset") {
    CHECK_THROWS_WITH_AS(generateScalarProfile(1.0, ScalarProfileConfig{std::nullopt, 3.0}),
                         "maxVelocity is required but was not set", ConfigError);
    CHECK_THROWS_WITH_AS(generateScalarProfile(1.0, ScalarProfileConfig{1.5, std::nullopt}),
                         "maxAccel is required but was not set", ConfigError);

    ProfileConfig config = validPathConfig();
    config.maxVelocity.reset();
    CHECK_THROWS_WITH_AS(generateTrajectory(kStraight, kNoKeyframes, false, config),
                         "maxVelocity is required but was not set", ConfigError);

    config = validPathConfig();
    config.maxAccel.reset();
    CHECK_THROWS_WITH_AS(generateTrajectory(kStraight, kNoKeyframes, false, config),
                         "maxAccel is required but was not set", ConfigError);

    config = validPathConfig();
    config.trackWidth.reset();
    CHECK_THROWS_WITH_AS(generateTrajectory(kStraight, kNoKeyframes, false, config),
                         "trackWidth is required but was not set", ConfigError);
}

TEST_CASE("a default-constructed config carries no robot limits") {
    const ProfileConfig path;
    CHECK_FALSE(path.maxVelocity.has_value());
    CHECK_FALSE(path.maxAccel.has_value());
    CHECK_FALSE(path.trackWidth.has_value());

    const ScalarProfileConfig scalar;
    CHECK_FALSE(scalar.maxVelocity.has_value());
    CHECK_FALSE(scalar.maxAccel.has_value());
}

TEST_CASE("dt and the RAMSETE gains keep their defaults") {
    const ProfileConfig path;
    CHECK(path.dt == doctest::Approx(0.01));
    CHECK(path.ramseteB == doctest::Approx(2.0));
    CHECK(path.ramseteZeta == doctest::Approx(0.7));
    CHECK(ScalarProfileConfig{}.dt == doctest::Approx(0.01));
}

TEST_CASE("generateTrajectory accepts valid limits") {
    CHECK_NOTHROW(generateTrajectory(kStraight, kNoKeyframes, false, validPathConfig()));
}
