#include "doctest.h"
#include "ramsete.hpp"

#include <cmath>
#include <vector>

TEST_CASE("a follower holding a steady turn drives the circle it is given") {
    // A robot holding v and omega for a step moves along an arc, not along the
    // heading it started the step with. Stepping along that heading drifts
    // outward by about v^2 / radius * dt^2 / 2 every step, which here is
    // 0.28 mm, and the follower's feedback then has to fight a drift the plan
    // never contained.
    constexpr double speed = 1.5;
    constexpr double radius = 0.4;
    constexpr double omega = speed / radius;
    constexpr double dt = 0.01;
    constexpr int kSamples = 150;

    std::vector<Pose> poses;
    std::vector<VelocityLayout> vels;
    for (int i = 0; i < kSamples; ++i) {
        const double t = i * dt;
        const double theta = omega * t;
        poses.push_back(Pose{ radius * std::sin(theta), radius * (1.0 - std::cos(theta)), theta });
        vels.push_back(VelocityLayout{ speed, omega, t });
    }

    RamseteFollower follower(poses, vels, 0.3, 2.0, 0.7, 0.0, dt, false);
    while (!follower.isFinished()) {
        follower.step();
    }

    const auto& followed = follower.getExecutedPoses();
    REQUIRE(followed.size() == poses.size());
    for (size_t i = 0; i < poses.size(); ++i) {
        CAPTURE(i);
        CHECK(std::hypot(followed[i].x - poses[i].x, followed[i].y - poses[i].y) < 1e-9);
    }
}
