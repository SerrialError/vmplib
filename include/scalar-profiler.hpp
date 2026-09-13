#pragma once

#include <optional>
#include <stdexcept>
#include <string>
#include <vector>
#include "scalar-profile.hpp"

// Motion limits for a 1D profile.
//
// Unit-agnostic: distance, velocity and acceleration may be in any units so
// long as they are consistent -- metres, metres/s, metres/s^2 for a straight
// drive, or radians, radians/s, radians/s^2 for a turret or arm. Gearing and
// motor conversions stay in the caller's robot code; this library only ever
// sees the axis it is profiling.
//
// maxVelocity and maxAccel describe the mechanism, so they start unset and must
// be supplied. dt defaults to one 10 ms V5 control loop.
struct ScalarProfileConfig {
    std::optional<double> maxVelocity;
    std::optional<double> maxAccel;
    double dt = 0.01;
};

// Thrown for a move that cannot be profiled as asked, such as one with a
// negative speed.
class MoveError : public std::runtime_error {
public:
    explicit MoveError(const std::string& what) : std::runtime_error(what) {}
};

// Profile a 1D move of the given signed distance: accelerate from startVel,
// hold under the velocity and acceleration limits and any keyframes, and land
// exactly on endVel at the end. Returns one (position, velocity, accel, time)
// sample every dt.
//
// A negative distance moves the other way: positions run from 0 down to
// distance, and velocity and acceleration carry the sign of the direction of
// travel. startVel, endVel and keyframe velocities are speeds along that
// direction, so none of them may be negative.
//
// keyframes pin a speed to a distance travelled from the start, between 0 and
// |distance|. They are sorted here, so the caller need not pre-sort them; fewer
// than two is treated as none. A distance of zero returns no samples.
//
// Throws ConfigError (config-error.hpp) if a limit in config is unset or not
// positive and finite, and MoveError if a speed is negative or a value is not
// finite.
std::vector<ScalarSample> generateScalarProfile(
    double distance,
    const ScalarProfileConfig& config,
    double startVel = 0.0,
    double endVel = 0.0,
    std::vector<ScalarKeyframe> keyframes = {});
