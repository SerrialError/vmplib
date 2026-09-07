#pragma once

#include <vector>
#include "types.hpp"

class TrapezoidalProfile {
public:
    TrapezoidalProfile(
        const std::vector<Point>& controlPts,
        double maxLinVel,
        double maxLinAccel,
        double trackWidth,
        double timeAccum,
        double startVel,
        double endVel,
        const std::vector<KeyframeVelocities>& keyframes,
        bool useKeyframes,
        double dt,
        // Arc length already travelled into this segment. A timestep almost
        // never lands exactly on a segment boundary, so the previous segment
        // hands over its overshoot here and the timestep grid stays uniform
        // across the join.
        double startArcLength = 0.0
    );

    // Advance one timestep. Returns (linear, angular, time)
    void start();
    void step();

    // True once t ≥ 1.0
    bool isFinished() const;

    // Distance the final step ran past the end of the segment. Feed this to the
    // next segment's startArcLength.
    double overshootArcLength() const;

    // Access generated path poses & velocities
    const std::vector<Pose>& getPoses() const;
    const std::vector<VelocityLayout>& getVelocities() const;


private:
    // Internal state
    double s_current_;
    double prev_t_;
    double time_accum_;
    double cur_speed_;
    double overshoot_;
    size_t step_count_;

    // Parameters
    // Owned rather than referenced: a profile routinely outlives the expression
    // that supplied its control points.
    std::vector<Point> control_;
    double max_lin_vel_;
    double max_lin_accel_;
    double track_width_;
    double exit_velocity_;
    bool use_keyframes_;
    double dt_;
    std::vector<KeyframeVelocities> keyframes_;

    // Total arc length of the segment; constant, so computed once.
    double total_length_;
    // Backstop so a non-advancing profile fails fast instead of looping forever.
    size_t max_steps_;

    // The velocity limit curve, sampled on a uniform grid in the Bezier
    // parameter. limit_v_ is the fastest the robot may travel at limit_s_ and
    // still respect the curvature and keyframe ceilings ahead of it without
    // ever exceeding max_lin_accel_ to slow down. Built up front because that
    // question cannot be answered from the current position alone; the
    // acceleration half of the profile is applied online in step().
    std::vector<double> limit_t_;
    std::vector<double> limit_s_;
    std::vector<double> limit_v_;

    // Accumulated output
    std::vector<Pose> poses_;
    std::vector<VelocityLayout> velocities_;

    // Helper methods
    void buildVelocityLimits();
    double computeCurvatureVelocityLimit(double t) const;
    double computeAccelerationLimit() const;
    double keyframeCeiling(double s, const std::vector<double>& keyframeS, size_t& idx) const;

    // The braking ramp expressed as a potential that is linear in arc length.
    // v^2 alone is the continuous ramp, which a fixed timestep cannot follow;
    // see buildVelocityLimits. These are mutual inverses.
    double brakingPotential(double v) const;
    double velocityAtPotential(double g) const;

    // Conversions against the sampled table above. Using it for both directions
    // keeps every arc length in the profiler on one consistent metric.
    double arcLengthAt(double t) const;
    double parameterAt(double s) const;
    double velocityAt(double s) const;
};
