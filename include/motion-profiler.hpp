#pragma once

#include <vector>
#include "types.hpp"

// Drivetrain and controller parameters. The defaults are the values this
// project was originally tuned against; a different robot overrides them
// rather than editing the source.
struct ProfileConfig {
    float maxVelocity = 1.8885160604f;   // m/s
    float maxAccel    = 4.12203073382f;  // m/s^2
    float trackWidth  = 0.29508135f;     // m
    // RAMSETE gains. b has units of 1/m^2 so that k = 2*zeta*sqrt(w^2 + b*v^2)
    // comes out in 1/s; zeta is the damping ratio and is dimensionless.
    float ramseteB    = 2.0f;
    float ramseteZeta = 0.7f;
    float dt          = 0.01f;           // s
};

// One entry per path segment. poses/velocities are the planned open-loop
// trajectory; followed* are what a RAMSETE follower achieves tracking it, which
// is what tells you whether the plan is trackable at all.
struct Trajectory {
    std::vector<std::vector<Pose>> poses;
    std::vector<std::vector<VelocityLayout>> velocities;
    std::vector<std::vector<Pose>> followedPoses;
    std::vector<std::vector<VelocityLayout>> followedVelocities;
};

// Plans a trajectory over the given cubic Bezier segments.
//
// keyframeList is indexed in parallel with controlPoints, one list per segment,
// and is ignored unless useKeyframes is set. Throws KeyframeError (bezier.hpp)
// if a keyframe cannot be placed on its segment.
Trajectory generateTrajectory(
    const std::vector<std::vector<Point>>& controlPoints,
    const std::vector<std::vector<KeyframeVelocitiesXandY>>& keyframeList,
    bool useKeyframes,
    const ProfileConfig& config = ProfileConfig{}
);
