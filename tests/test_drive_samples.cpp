#include "doctest.h"
#include "config-error.hpp"
#include "drive-samples.hpp"

#include <cmath>
#include <limits>
#include <vector>

namespace {

constexpr double kTrackWidth = 0.3;
constexpr double kDt = 0.01;

const std::vector<std::vector<Point>> kStraight = {
    {{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}}};

// Two S-curves joined end to end, so the path turns both ways and has a join
// to difference across.
const std::vector<std::vector<Point>> kTwoSegments = {
    {{0.0, 0.0}, {0.5, 0.0}, {0.5, 1.0}, {1.0, 1.0}},
    {{1.0, 1.0}, {1.5, 1.0}, {1.5, 0.0}, {2.0, 0.0}}};

ProfileConfig config() {
    ProfileConfig c;
    c.maxVelocity = 1.5;
    c.maxAccel = 3.0;
    c.trackWidth = kTrackWidth;
    c.dt = kDt;
    return c;
}

Trajectory plan(const std::vector<std::vector<Point>>& path) {
    const std::vector<std::vector<KeyframeVelocitiesXandY>> noKeyframes(path.size());
    return generateTrajectory(path, noKeyframes, false, config());
}

} // namespace

TEST_CASE("a straight path drives both sides at the robot's speed") {
    const std::vector<DriveSample> samples = driveSamples(plan(kStraight), config());
    REQUIRE(samples.size() > 1);
    for (const DriveSample& s : samples) {
        CAPTURE(s.time);
        CHECK(s.angular == doctest::Approx(0.0));
        CHECK(s.leftVelocity == doctest::Approx(s.linear));
        CHECK(s.rightVelocity == doctest::Approx(s.linear));
    }
}

TEST_CASE("side speeds average to the robot's speed and differ by its turn rate") {
    const std::vector<DriveSample> samples = driveSamples(plan(kTwoSegments), config());
    bool turnedLeft = false;
    bool turnedRight = false;
    for (const DriveSample& s : samples) {
        CAPTURE(s.time);
        CHECK((s.leftVelocity + s.rightVelocity) / 2.0 == doctest::Approx(s.linear));
        CHECK((s.rightVelocity - s.leftVelocity) / kTrackWidth == doctest::Approx(s.angular));
        turnedLeft = turnedLeft || s.angular > 0.1;
        turnedRight = turnedRight || s.angular < -0.1;
    }
    CHECK(turnedLeft);
    CHECK(turnedRight);
}

TEST_CASE("every planned sample becomes one drive sample, dt apart") {
    const Trajectory traj = plan(kTwoSegments);
    REQUIRE(traj.velocities.size() == 2);
    const std::vector<DriveSample> samples = driveSamples(traj, config());

    REQUIRE(samples.size() == traj.velocities[0].size() + traj.velocities[1].size());
    CHECK(samples.front().time == 0.0);
    for (size_t i = 1; i < samples.size(); ++i) {
        CAPTURE(i);
        CHECK(samples[i].time - samples[i - 1].time == doctest::Approx(kDt).epsilon(1e-6));
    }
}

TEST_CASE("each side's acceleration carries it to the next sample, across the join too") {
    const Trajectory traj = plan(kTwoSegments);
    REQUIRE(traj.velocities.size() == 2);
    const std::vector<DriveSample> samples = driveSamples(traj, config());
    REQUIRE(samples.size() > 1);

    for (size_t i = 0; i + 1 < samples.size(); ++i) {
        CAPTURE(i);
        CHECK(samples[i].leftVelocity + samples[i].leftAccel * kDt ==
              doctest::Approx(samples[i + 1].leftVelocity));
        CHECK(samples[i].rightVelocity + samples[i].rightAccel * kDt ==
              doctest::Approx(samples[i + 1].rightVelocity));
    }
    // Nothing follows the last sample to accelerate towards.
    CHECK(samples.back().leftAccel == 0.0);
    CHECK(samples.back().rightAccel == 0.0);
}

TEST_CASE("an empty trajectory gives no drive samples") {
    CHECK(driveSamples(Trajectory{}, config()).empty());
}

TEST_CASE("driveSamples rejects a track width or dt that is not positive and finite") {
    const Trajectory traj = plan(kStraight);

    ProfileConfig unset = config();
    unset.trackWidth.reset();
    CHECK_THROWS_AS(driveSamples(traj, unset), ConfigError);

    for (double bad : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::infinity()}) {
        CAPTURE(bad);
        ProfileConfig badTrack = config();
        badTrack.trackWidth = bad;
        CHECK_THROWS_AS(driveSamples(traj, badTrack), ConfigError);

        ProfileConfig badDt = config();
        badDt.dt = bad;
        CHECK_THROWS_AS(driveSamples(traj, badDt), ConfigError);
    }
}

TEST_CASE("no side is asked to accelerate faster than the limit") {
    // The path in examples/path-points.txt with the README's limits. Limiting
    // only the centre asked its outer side for 7.9 m/s^2 against 4.122.
    const std::vector<std::vector<Point>> example = {
        {{-0.586, -0.41}, {-0.586, -0.201}, {-0.997, 0.335}, {-0.997, 0.544}}};
    ProfileConfig exampleConfig;
    exampleConfig.maxVelocity = 1.8885;
    exampleConfig.maxAccel = 4.1220;
    exampleConfig.trackWidth = 0.2951;

    struct Case {
        const char* name;
        const std::vector<std::vector<Point>>* path;
        ProfileConfig config;
    };
    const Case cases[] = {
        {"the example path", &example, exampleConfig},
        {"two S-curves", &kTwoSegments, config()},
    };

    for (const Case& c : cases) {
        CAPTURE(c.name);
        const std::vector<std::vector<KeyframeVelocitiesXandY>> noKeyframes(c.path->size());
        const Trajectory traj = generateTrajectory(*c.path, noKeyframes, false, c.config);
        const std::vector<DriveSample> samples = driveSamples(traj, c.config);
        REQUIRE(samples.size() > 1);

        const double limit = *c.config.maxAccel * (1.0 + 1e-9);
        for (const DriveSample& s : samples) {
            CAPTURE(s.time);
            CHECK(std::fabs(s.leftAccel) <= limit);
            CHECK(std::fabs(s.rightAccel) <= limit);
        }
    }
}
