#pragma once

#include <vector>

// A target velocity pinned to a distance along the profile.
struct ScalarKeyframe {
    double velocity;
    double distance;
};

// One profiled sample along a distance axis. Carried in double like the rest of
// the library's internal state; narrow to float only at the output boundary.
struct ScalarSample {
    double position;   // arc length from the start of the profile
    double velocity;
    double accel;      // (velocity - previous velocity) / dt; 0 at the first sample
    double time;
};

// Forward/backward velocity profiling over a scalar arc length.
//
// This is the 1D core the Bezier profiler is built on. It knows nothing about
// x, y, or heading: only a distance axis, a velocity ceiling sampled along it,
// the acceleration limit, and the start/exit velocities. Given those it runs
// the backward deceleration pass up front and the forward acceleration pass
// online, one timestep at a time, and emits (position, velocity, accel, time)
// samples.
//
// The velocity ceiling is supplied as a value per distance node
// (ceilingDistances / ceilingVelocities, parallel arrays). A 1D caller passes
// its top speed at every node; the Bezier profiler passes the lower of the top
// speed and the curvature limit. Distance-indexed keyframes are folded in on
// top of that ceiling here, since keyframes are a scalar constraint.
class ScalarProfile {
public:
    ScalarProfile(
        double distance,
        // Distance grid the velocity ceiling is sampled on. Monotonically
        // increasing, with ceilingDistances.front() == 0 and .back() == distance;
        // ceilingVelocities is parallel to it.
        std::vector<double> ceilingDistances,
        // Hard velocity ceiling at each grid node, before keyframes and the
        // acceleration/braking passes.
        std::vector<double> ceilingVelocities,
        double maxAccel,
        double startVel,
        double endVel,
        double dt,
        // Distance-indexed velocity targets. Fewer than two is treated as none,
        // since a single keyframe has nothing to interpolate against.
        std::vector<ScalarKeyframe> keyframes,
        // Arc length already travelled into the path before this profile begins.
        // A timestep almost never lands exactly on a path boundary, so a caller
        // stitching profiles together hands over the previous one's overshoot
        // here and the timestep grid stays uniform across the join.
        double startDistance = 0.0,
        // Wall-clock time the first sample is stamped with; subsequent samples
        // advance by dt.
        double timeAccum = 0.0
    );

    // Emit the sample at the starting position. Optional: a caller stitching
    // profiles together skips it on every profile but the first, where the
    // start pose is the join the previous profile's final sample already covers.
    void start();

    // Advance one timestep and emit a sample.
    void step();

    // True once the profile has reached the end of the distance axis.
    bool isFinished() const;

    // Distance the final step ran past the end of the path. Hand this to the
    // next profile's startDistance to keep the timestep grid continuous.
    double overshootDistance() const;

    const std::vector<ScalarSample>& samples() const;

private:
    // Fold the keyframes into the supplied ceiling, then run the backward pass.
    void buildVelocityLimits(const std::vector<double>& ceilingVelocities,
                             const std::vector<ScalarKeyframe>& keyframes);

    // Speed reachable in one timestep from the current speed under the
    // acceleration limit.
    double accelerationLimit() const;

    // Velocity cap imposed by the keyframes bracketing s. idx is carried across
    // calls so a monotonic sweep does not rescan the list.
    double keyframeCeiling(double s, const std::vector<ScalarKeyframe>& keyframes,
                           size_t& idx) const;

    // The braking ramp expressed as a potential that is linear in arc length.
    // v^2 alone is the continuous ramp, which a fixed timestep cannot follow;
    // see buildVelocityLimits. These are mutual inverses.
    double brakingPotential(double v) const;
    double velocityAtPotential(double g) const;

    // The velocity ceiling at an arbitrary arc length, interpolated in the
    // braking potential the backward pass is linear in.
    double velocityAt(double s) const;

    // Parameters
    double distance_;
    double max_accel_;
    double exit_velocity_;
    double dt_;

    // The velocity limit curve: limit_v_[i] is the fastest the robot may travel
    // at limit_s_[i] and still honour every ceiling ahead of it without ever
    // exceeding max_accel_ to slow down. Built once up front; the acceleration
    // half of the profile is applied online in step().
    std::vector<double> limit_s_;
    std::vector<double> limit_v_;

    // Internal state
    double s_current_;
    double cur_speed_;
    double time_accum_;
    double overshoot_;
    size_t step_count_;
    size_t max_steps_;

    std::vector<ScalarSample> samples_;
};
