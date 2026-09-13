#include "doctest.h"
#include "config-error.hpp"
#include "motion-profiler.hpp"
#include "scalar-profiler.hpp"

#include <limits>
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

TEST_CASE("generateTrajectory accepts valid limits") {
    CHECK_NOTHROW(generateTrajectory(kStraight, kNoKeyframes, false, validPathConfig()));
}
