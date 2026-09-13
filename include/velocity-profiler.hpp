#pragma once

#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

// Motion limits for a velocity profile. Unit-agnostic like ScalarProfileConfig:
// rad/s and rad/s^2 for a flywheel, or whatever the robot code works in.
//
// maxAccel describes the mechanism, so it starts unset and must be supplied.
// maxVelocity is optional; when set, a target faster than it is rejected rather
// than clamped. dt defaults to one 10 ms V5 control loop.
struct VelocityProfileConfig {
    std::optional<double> maxVelocity;
    std::optional<double> maxAccel;
    double dt = 0.01;
};

// A velocity to ramp to, then hold for hold seconds once it is reached.
struct VelocityTarget {
    double velocity;
    double hold = 0.0;
};

// One profiled sample in time.
struct VelocitySample {
    double velocity;
    double accel;   // (velocity - previous velocity) / dt; 0 at the first sample
    double time;
};

// Thrown for a velocity target that cannot be profiled as asked.
class VelocityTargetError : public std::runtime_error {
public:
    explicit VelocityTargetError(const std::string& what) : std::runtime_error(what) {}
};

// Profile a mechanism commanded by speed rather than position -- a flywheel,
// roller or intake. Starting from startVelocity at time 0, ramp to each target in
// turn at the acceleration limit, landing exactly on it, then hold it for at
// least its hold time, rounded up to a whole dt. Velocities are signed, so a ramp
// may pass through zero. Returns one sample every dt; no targets returns no
// samples.
//
// Throws ConfigError (config-error.hpp) if maxAccel is unset, or if a limit that
// is set or dt is not positive and finite. Throws VelocityTargetError if a
// velocity is not finite or is faster than maxVelocity, or a hold is negative or
// not finite.
std::vector<VelocitySample> generateVelocityProfile(
    const VelocityProfileConfig& config,
    const std::vector<VelocityTarget>& targets,
    double startVelocity = 0.0);
