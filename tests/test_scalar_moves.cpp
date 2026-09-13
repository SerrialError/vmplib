#include "doctest.h"
#include "config-error.hpp"
#include "scalar-profiler.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace {

constexpr double kMaxVel = 1.5;
constexpr double kMaxAccel = 3.0;
constexpr double kDt = 0.01;

const ScalarProfileConfig kConfig{ kMaxVel, kMaxAccel, kDt };

// What every sequence must hold however its moves are arranged: one sample per
// dt, never above top speed, no step changing velocity by more than the
// acceleration limit allows -- joins included -- and velocity pointing the way
// the mechanism is actually moving.
void checkKinematics(const std::vector<ScalarSample>& samples) {
    REQUIRE(samples.size() > 1);
    const double maxDelta = kMaxAccel * kDt + 1e-6;
    for (size_t i = 0; i < samples.size(); ++i) {
        CAPTURE(i);
        CHECK(std::fabs(samples[i].velocity) <= kMaxVel + 1e-6);
        if (i == 0) {
            continue;
        }
        CHECK(samples[i].time - samples[i - 1].time == doctest::Approx(kDt).epsilon(1e-6));
        CHECK(std::fabs(samples[i].velocity - samples[i - 1].velocity) <= maxDelta);
        CHECK((samples[i].position - samples[i - 1].position) * samples[i].velocity >= -1e-12);
    }
}

} // namespace

TEST_CASE("a single move matches generateScalarProfile over the same distance") {
    const std::vector<ScalarKeyframe> travelled = {{kMaxVel, 0.0}, {0.3, 1.0}, {kMaxVel, 2.0}};

    SUBCASE("forwards") {
        const ScalarMove move{2.5, 0.0, {{0.5, kMaxVel}, {1.5, 0.3}, {2.5, kMaxVel}}};
        const auto moves = generateScalarMoves(kConfig, 0.5, {move});
        const auto single = generateScalarProfile(2.0, kConfig, 0.0, 0.0, travelled);
        REQUIRE(moves.size() == single.size());
        for (size_t i = 0; i < moves.size(); ++i) {
            CAPTURE(i);
            CHECK(moves[i].position == doctest::Approx(0.5 + single[i].position));
            CHECK(moves[i].velocity == doctest::Approx(single[i].velocity));
        }
    }
    SUBCASE("backwards") {
        const ScalarMove move{0.5, 0.0, {{2.5, kMaxVel}, {1.5, 0.3}, {0.5, kMaxVel}}};
        const auto moves = generateScalarMoves(kConfig, 2.5, {move});
        const auto single = generateScalarProfile(-2.0, kConfig, 0.0, 0.0, travelled);
        REQUIRE(moves.size() == single.size());
        for (size_t i = 0; i < moves.size(); ++i) {
            CAPTURE(i);
            CHECK(moves[i].position == doctest::Approx(2.5 + single[i].position));
            CHECK(moves[i].velocity == doctest::Approx(single[i].velocity));
        }
    }
}

TEST_CASE("a reversal comes to rest at the turning point") {
    // Raise a lift to 0.8, then lower it to 0.2.
    const auto samples = generateScalarMoves(kConfig, 0.0, {{0.8}, {0.2}});
    checkKinematics(samples);

    CHECK(samples.front().position == 0.0);
    CHECK(samples.front().velocity == 0.0);
    CHECK(samples.back().position == 0.2);
    CHECK(samples.back().velocity == doctest::Approx(0.0));

    const auto top = std::max_element(samples.begin(), samples.end(),
                                      [](const ScalarSample& a, const ScalarSample& b) {
                                          return a.position < b.position;
                                      });
    CHECK(top->position == doctest::Approx(0.8));
    CHECK(top->velocity == doctest::Approx(0.0));
    for (auto it = samples.begin(); it != samples.end(); ++it) {
        CHECK((it <= top ? it->velocity >= 0.0 : it->velocity <= 0.0));
    }
}

TEST_CASE("moves in one direction brake to each join's end velocity") {
    // Slow to 0.3 at the 1.0 join, then carry on to a stop at 2.0.
    const auto samples = generateScalarMoves(kConfig, 0.0, {{1.0, 0.3}, {2.0}});
    checkKinematics(samples);
    CHECK(samples.back().position == 2.0);
    CHECK(samples.back().velocity == doctest::Approx(0.0));

    // Before the join the profile must always be able to brake to 0.3 by it. The
    // continuous ramp is the ceiling; the profile rides the discrete one below it.
    bool sawJoin = false;
    for (const ScalarSample& s : samples) {
        if (s.position >= 1.0) {
            sawJoin = true;
            break;
        }
        CAPTURE(s.position);
        CHECK(s.velocity <= std::sqrt(0.3 * 0.3 + 2.0 * kMaxAccel * (1.0 - s.position)) + 1e-4);
    }
    CHECK(sawJoin);
}

TEST_CASE("a short move after a fast join is braked for before the join") {
    // Move 1 asks to reach the join at top speed, but move 2 is only 5 cm and
    // must stop: the mechanism has to start braking inside move 1.
    const auto samples = generateScalarMoves(kConfig, 0.0, {{1.5, kMaxVel}, {1.55}});
    checkKinematics(samples);
    CHECK(samples.back().position == 1.55);
    CHECK(samples.back().velocity == doctest::Approx(0.0));
    for (const ScalarSample& s : samples) {
        CAPTURE(s.position);
        CHECK(s.velocity <= std::sqrt(2.0 * kMaxAccel * (1.55 - s.position)) + 1e-4);
    }
}

TEST_CASE("a long mixed sequence holds every kinematic limit") {
    const std::vector<ScalarMove> moves = {
        {0.6, 0.8},
        {1.4, 0.0, {{0.6, kMaxVel}, {1.0, 0.2}, {1.4, kMaxVel}}},
        {-0.3},
        {-0.2, 0.4},
        {0.9, 0.0},
    };
    const auto samples = generateScalarMoves(kConfig, 0.0, moves);
    checkKinematics(samples);
    CHECK(samples.back().position == 0.9);
    CHECK(samples.back().velocity == doctest::Approx(0.0));
}

TEST_CASE("a move sequence that cannot be profiled as asked throws") {
    CHECK_THROWS_AS(generateScalarMoves(kConfig, 0.5, {{0.5}}), MoveError);
    CHECK_THROWS_AS(generateScalarMoves(kConfig, 0.0, {{1.0, -0.1}}), MoveError);
    CHECK_THROWS_AS(generateScalarMoves(kConfig, 0.0, {{1.0, 0.0, {{1.2, 0.5}, {0.5, 0.5}}}}),
                    MoveError);
    CHECK_THROWS_AS(generateScalarMoves(kConfig, 0.0, {{1.0, 0.0, {{0.2, -0.5}, {0.5, 0.5}}}}),
                    MoveError);
    // Reversing at speed would need infinite deceleration.
    CHECK_THROWS_AS(generateScalarMoves(kConfig, 0.0, {{1.0, 0.5}, {0.0}}), MoveError);
    // 80 m at 1 m/s outlasts the watchdog, even split across two moves.
    CHECK_THROWS_AS(generateScalarMoves(ScalarProfileConfig{1.0, 3.0}, 0.0, {{40.0}, {80.0}}),
                    MoveError);
    CHECK_THROWS_AS(generateScalarMoves(ScalarProfileConfig{}, 0.0, {{1.0}}), ConfigError);
}

TEST_CASE("no moves yield no samples") {
    CHECK(generateScalarMoves(kConfig, 0.0, {}).empty());
}
