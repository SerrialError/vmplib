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

TEST_CASE("curvature saturates high at a cusp, not to zero") {
    // P0 == P1 makes r'(0) vanish, so the curvature denominator collapses.
    const std::vector<Point> cusp = {{0.f, 0.f}, {0.f, 0.f}, {1.f, 1.f}, {2.f, 0.f}};

    const float kappa = unsignedCurvature(cusp, 0.f);
    CHECK(kappa > 100.f);

    // The curvature speed limit must therefore collapse toward zero rather
    // than leaving the robot unrestricted at the sharpest point on the path.
    const float trackWidth = 0.295f;
    const float radius = 1.f / kappa;
    const float limit = radius / (radius + trackWidth / 2.f);
    CHECK(limit < 0.01f);
}

TEST_CASE("composite quadrature is more accurate than a single panel") {
    // A long curve where the integrand varies sharply: a single 5-point panel
    // over [0,1] measurably underestimates the length.
    const std::vector<Point> wiggly = {{0.f, 0.f}, {6.f, 5.f}, {-5.f, 5.f}, {1.f, 0.f}};

    // Reference: fine-grained polyline through many samples.
    double reference = 0.0;
    const int samples = 20000;
    Pose prev = findXandY(wiggly, 0.f);
    for (int i = 1; i <= samples; ++i) {
        const Pose cur = findXandY(wiggly, static_cast<float>(i) / samples);
        const double dx = cur.x - prev.x;
        const double dy = cur.y - prev.y;
        reference += std::sqrt(dx * dx + dy * dy);
        prev = cur;
    }

    const float single = arcLength(wiggly, 0.f, 1.f);
    const float composite = sFunction(wiggly, 1.f);

    CHECK(std::fabs(composite - reference) < std::fabs(single - reference));
    CHECK(composite == doctest::Approx(reference).epsilon(1e-3));
}

TEST_CASE("findTForS honours the initial guess without changing the root") {
    const float total = sFunction(kQuarterCircle, 1.f);
    const float target = total * 0.7f;

    // Every seed must land on the same root.
    const float fromLow = findTForS(kQuarterCircle, 0.f, target, 0.05f);
    const float fromMid = findTForS(kQuarterCircle, 0.f, target, 0.5f);
    const float fromHigh = findTForS(kQuarterCircle, 0.f, target, 0.95f);

    CHECK(fromLow == doctest::Approx(fromMid).epsilon(1e-3));
    CHECK(fromHigh == doctest::Approx(fromMid).epsilon(1e-3));
    CHECK(sFunction(kQuarterCircle, fromMid) == doctest::Approx(target).epsilon(1e-3));
}

TEST_CASE("findTForS saturates at t=1 when the target is past the end") {
    const float total = sFunction(kStraight, 1.f);
    CHECK(findTForS(kStraight, 0.f, total * 2.f) == doctest::Approx(1.0).epsilon(1e-5));
}

TEST_CASE("findTForS falls back to bisection near a cusp") {
    // r'(0) = 0, so Newton's derivative vanishes at the seed.
    const std::vector<Point> cusp = {{0.f, 0.f}, {0.f, 0.f}, {1.f, 1.f}, {2.f, 0.f}};
    const float total = sFunction(cusp, 1.f);
    REQUIRE(total > 0.f);

    const float t = findTForS(cusp, 0.f, total * 0.5f, 0.0f);
    CHECK(t > 0.f);
    CHECK(t < 1.f);
    CHECK(sFunction(cusp, t) == doctest::Approx(total * 0.5f).epsilon(1e-2));
}

TEST_CASE("projectOntoCurve recovers the parameter of an on-curve point") {
    for (int i = 0; i <= 10; ++i) {
        const float expected = i / 10.f;
        const Pose p = findXandY(kQuarterCircle, expected);

        float residual = -1.f;
        const float t = projectOntoCurve(kQuarterCircle, p.x, p.y, &residual);

        CAPTURE(expected);
        CHECK(t == doctest::Approx(expected).epsilon(1e-2));
        CHECK(residual == doctest::Approx(0.0).epsilon(1e-4));
    }
}

TEST_CASE("projectOntoCurve reports how far off-path a point is") {
    // The quarter circle is centred on the origin with unit radius, so a point
    // at radius 1.2 along the 45-degree ray sits 0.2 outside the curve.
    const float diag = 1.2f / std::sqrt(2.f);

    float residual = -1.f;
    const float t = projectOntoCurve(kQuarterCircle, diag, diag, &residual);

    CHECK(t == doctest::Approx(0.5).epsilon(0.05));
    CHECK(residual == doctest::Approx(0.2).epsilon(0.02));
}

TEST_CASE("projectOntoCurve uses y, not just x") {
    // Both points share an x coordinate but sit at opposite ends of the curve.
    // An x-only root solve cannot tell them apart.
    const std::vector<Point> arch = {{0.f, 0.f}, {0.f, 2.f}, {2.f, 2.f}, {2.f, 0.f}};

    const Pose low = findXandY(arch, 0.15f);
    const Pose high = findXandY(arch, 0.85f);

    const float tLow = projectOntoCurve(arch, low.x, low.y);
    const float tHigh = projectOntoCurve(arch, high.x, high.y);

    CHECK(tLow == doctest::Approx(0.15).epsilon(0.05));
    CHECK(tHigh == doctest::Approx(0.85).epsilon(0.05));
    CHECK(tLow < tHigh);
}

TEST_CASE("convertToTFrame keeps on-path keyframes ordered along the curve") {
    std::vector<KeyframeVelocitiesXandY> xy;
    for (int i = 1; i <= 4; ++i) {
        const Pose p = findXandY(kQuarterCircle, i / 5.f);
        xy.push_back({p.x, p.y, 0.5f * i});
    }

    const std::vector<KeyframeVelocities> t = convertToTFrame(kQuarterCircle, xy);
    REQUIRE(t.size() == xy.size());

    for (size_t i = 0; i < t.size(); ++i) {
        CHECK(t[i].velocity == doctest::Approx(xy[i].velocity));
        CHECK(t[i].t == doctest::Approx((i + 1) / 5.f).epsilon(0.05));
        if (i > 0) CHECK(t[i].t > t[i - 1].t);
    }
}

TEST_CASE("convertToTFrame rejects a keyframe that is off the path") {
    // 0.2 outside a unit-radius arc, well past the 0.05 tolerance.
    const float diag = 1.2f / std::sqrt(2.f);
    const std::vector<KeyframeVelocitiesXandY> xy = {{diag, diag, 1.f}};

    CHECK_THROWS_AS(convertToTFrame(kQuarterCircle, xy), KeyframeError);
}

TEST_CASE("convertToTFrame rejects keyframes that run backwards along the path") {
    const Pose early = findXandY(kQuarterCircle, 0.25f);
    const Pose late = findXandY(kQuarterCircle, 0.75f);

    const std::vector<KeyframeVelocitiesXandY> backwards = {
        {late.x, late.y, 1.f}, {early.x, early.y, 1.f}};
    CHECK_THROWS_AS(convertToTFrame(kQuarterCircle, backwards), KeyframeError);

    // The same two keyframes in path order are accepted.
    const std::vector<KeyframeVelocitiesXandY> forwards = {
        {early.x, early.y, 1.f}, {late.x, late.y, 1.f}};
    CHECK_NOTHROW(convertToTFrame(kQuarterCircle, forwards));
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
