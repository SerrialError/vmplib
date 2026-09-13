#pragma once

#include <optional>
#include <vector>
#include "types.hpp"

// Drivetrain and controller parameters.
//
// The limits that describe a particular robot start unset and must be supplied;
// generateTrajectory throws ConfigError if any is missing. dt and the RAMSETE
// gains are not specific to one robot, so they keep defaults.
struct ProfileConfig {
    std::optional<double> maxVelocity;   // m/s
    std::optional<double> maxAccel;      // m/s^2
    std::optional<double> trackWidth;    // m
    // RAMSETE gains. b has units of 1/m^2 so that k = 2*zeta*sqrt(w^2 + b*v^2)
    // comes out in 1/s; zeta is the damping ratio and is dimensionless.
    double ramseteB    = 2.0;
    double ramseteZeta = 0.7;
    double dt          = 0.01;           // s
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
// if a keyframe cannot be placed on its segment, and ConfigError
// (config-error.hpp) if a limit in config is unset or not positive and finite.
Trajectory generateTrajectory(
    const std::vector<std::vector<Point>>& controlPoints,
    const std::vector<std::vector<KeyframeVelocitiesXandY>>& keyframeList,
    bool useKeyframes,
    const ProfileConfig& config
);
