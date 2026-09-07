#pragma once

#include <vector>
#include "scalar-profile.hpp"

// Motion limits for a 1D profile.
//
// Unit-agnostic: distance, velocity and acceleration may be in any units so
// long as they are consistent -- metres, metres/s, metres/s^2 for a straight
// drive, or radians, radians/s, radians/s^2 for a turret or arm. Gearing and
// motor conversions stay in the caller's robot code; this library only ever
// sees the axis it is profiling.
struct ScalarProfileConfig {
    double maxVelocity;
    double maxAccel;
    double dt;
};

// Profile a straight 1D move of the given distance: accelerate from startVel,
// hold under the velocity and acceleration limits and any keyframes, and land
// exactly on endVel at the end. Returns one (position, velocity, accel, time)
// sample every dt.
//
// keyframes pin a velocity to a distance along the move. They are sorted by
// distance here, so the caller need not pre-sort them; fewer than two is
// treated as none. A distance of zero or less returns no samples.
std::vector<ScalarSample> generateScalarProfile(
    double distance,
    const ScalarProfileConfig& config,
    double startVel = 0.0,
    double endVel = 0.0,
    std::vector<ScalarKeyframe> keyframes = {});
