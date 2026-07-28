#include "doctest.h"
#include "bezier.hpp"

#include <cmath>
#include <vector>

namespace {

// Control points evenly spaced along +x: the curve reduces to x(t) = 3t, y = 0.
const std::vector<Point> kStraight = {{0.f, 0.f}, {1.f, 0.f}, {2.f, 0.f}, {3.f, 0.f}};

// Turns left; curvature at t=0 is hand-derived below.
const std::vector<Point> kLeftTurn = {{0.f, 0.f}, {1.f, 0.f}, {1.f, 1.f}, {0.f, 1.f}};

// Mirror of kLeftTurn about y=0.
const std::vector<Point> kRightTurn = {{0.f, 0.f}, {1.f, 0.f}, {1.f, -1.f}, {0.f, -1.f}};

// Standard 4-point cubic approximation of a unit quarter circle.
const float kCircleK = 0.5522847498f;
const std::vector<Point> kQuarterCircle = {
    {1.f, 0.f}, {1.f, kCircleK}, {kCircleK, 1.f}, {0.f, 1.f}};

} // namespace

TEST_CASE("straight-line arc length equals euclidean distance") {
    CHECK(arcLength(kStraight, 0.f, 1.f) == doctest::Approx(3.0).epsilon(1e-4));
    CHECK(sFunction(kStraight, 1.f) == doctest::Approx(3.0).epsilon(1e-4));
    CHECK(sFunction(kStraight, 0.5f) == doctest::Approx(1.5).epsilon(1e-4));
    CHECK(sFunction(kStraight, 0.f) == doctest::Approx(0.0).epsilon(1e-6));
}

TEST_CASE("arc length is monotonically increasing in t") {
    float prev = -1.f;
    for (int i = 0; i <= 40; ++i) {
        const float s = sFunction(kQuarterCircle, i / 40.f);
        CHECK(s > prev);
        prev = s;
    }
}

TEST_CASE("arc length of a quarter circle approximates R*pi/2") {
    // The cubic approximation is accurate to well under 1% in length.
    const float expected = static_cast<float>(M_PI) / 2.f;
    CHECK(sFunction(kQuarterCircle, 1.f) == doctest::Approx(expected).epsilon(0.01));
}

TEST_CASE("straight line has zero curvature") {
    for (float t : {0.f, 0.25f, 0.5f, 0.75f, 1.f}) {
        CHECK(signedCurvature(kStraight, t) == doctest::Approx(0.0).epsilon(1e-5));
        CHECK(unsignedCurvature(kStraight, t) == doctest::Approx(0.0).epsilon(1e-5));
    }
}

TEST_CASE("curvature at t=0 matches the hand-derived value") {
    // r'(0)  = 3(P1 - P0)              = (3, 0)
    // r''(0) = 6(P2 - 2*P1 + P0)       = (-6, 6)
    // kappa  = (x'y'' - y'x'') / |r'|^3 = 18 / 27
    CHECK(signedCurvature(kLeftTurn, 0.f) == doctest::Approx(2.0 / 3.0).epsilon(1e-4));
}

TEST_CASE("curvature sign follows turn direction") {
    const float left = signedCurvature(kLeftTurn, 0.5f);
    const float right = signedCurvature(kRightTurn, 0.5f);

    CHECK(left > 0.f);
    CHECK(right < 0.f);
    CHECK(left == doctest::Approx(-right).epsilon(1e-5));

    CHECK(unsignedCurvature(kLeftTurn, 0.5f) == doctest::Approx(std::fabs(left)).epsilon(1e-5));
    CHECK(unsignedCurvature(kRightTurn, 0.5f) == doctest::Approx(std::fabs(right)).epsilon(1e-5));
}

TEST_CASE("quarter-circle curvature is close to 1/R") {
    // The endpoint curvature of this approximation is ~0.978, not exactly 1;
    // the whole curve stays within a few percent of unit curvature.
    for (int i = 0; i <= 10; ++i) {
        const float kappa = unsignedCurvature(kQuarterCircle, i / 10.f);
        CHECK(kappa == doctest::Approx(1.0).epsilon(0.05));
    }
}

TEST_CASE("findTForS inverts sFunction on a straight line") {
    // s(t) = 3t, so advancing 1.5 from the start lands exactly at t = 0.5.
    CHECK(findTForS(kStraight, 0.f, 1.5f) == doctest::Approx(0.5).epsilon(1e-4));
    CHECK(findTForS(kStraight, 1.5f, 1.5f) == doctest::Approx(1.0).epsilon(1e-4));
    CHECK(findTForS(kStraight, 0.f, 0.75f) == doctest::Approx(0.25).epsilon(1e-4));
}

TEST_CASE("findTForS round-trips against sFunction on a curve") {
    const float total = sFunction(kQuarterCircle, 1.f);
    for (int i = 1; i < 10; ++i) {
        const float target = total * (i / 10.f);
        const float t = findTForS(kQuarterCircle, 0.f, target);
        CHECK(sFunction(kQuarterCircle, t) == doctest::Approx(target).epsilon(1e-3));
    }
}

TEST_CASE("findXandY matches the Bezier definition at the endpoints") {
    const Pose start = findXandY(kQuarterCircle, 0.f);
    const Pose end = findXandY(kQuarterCircle, 1.f);

    CHECK(start.x == doctest::Approx(1.0).epsilon(1e-5));
    CHECK(start.y == doctest::Approx(0.0).epsilon(1e-5));
    CHECK(end.x == doctest::Approx(0.0).epsilon(1e-5));
    CHECK(end.y == doctest::Approx(1.0).epsilon(1e-5));

    // Heading at the start of a quarter circle beginning at (R,0) points +y.
    CHECK(start.theta == doctest::Approx(M_PI / 2.0).epsilon(1e-4));
}
