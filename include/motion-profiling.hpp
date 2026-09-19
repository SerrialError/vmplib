#pragma once

#include <optional>
#include <vector>
#include "types.hpp"
#include "scalar-profile.hpp"

// Velocity profile along a cubic Bezier segment.
//
// It is a thin adapter over ScalarProfile: the shared 1D core does the
// backward/forward velocity passes on arc length, and this class supplies the
// things that are specific to a 2D path -- the curvature velocity ceiling, the
// curvature that limits each side's acceleration, and the map from arc length
// back to a pose (x, y, theta) and angular velocity.
class BezierPathProfile {
public:
    BezierPathProfile(
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
        double startArcLength = 0.0,
        // The previous segment's endCurvature(), which this segment's first step
        // turns from. Unset on the first segment, whose start sample is its own.
        std::optional<double> previousCurvature = std::nullopt
    );

    // Emit the sample at the start pose. Optional: on a multi-segment path only
    // the first segment calls it, since every later start pose is the join the
    // previous segment's final sample already covers.
    void start();

    // Advance one timestep and emit a pose and velocity.
    void step();

    // True once the profile has reached the end of the segment.
    bool isFinished() const;

    // Distance the final step ran past the end of the segment. Feed this to the
    // next segment's startArcLength.
    double overshootArcLength() const;

    // Curvature at the last sample. Feed this to the next segment's
    // previousCurvature.
    double endCurvature() const;

    // Access generated path poses & velocities
    const std::vector<Pose>& getPoses() const;
    const std::vector<VelocityLayout>& getVelocities() const;

private:
    // Arc length up to Bezier parameter t, read off the sampled table below.
    double arcLengthAt(double t) const;
    // Inverse of arcLengthAt: the Bezier parameter at a given arc length.
    double parameterAt(double s) const;
    // Turn every ScalarProfile sample not yet converted into a pose and velocity.
    void emitSamples();

    // Owned rather than referenced: a profile routinely outlives the expression
    // that supplied its control points.
    std::vector<Point> control_;
    double max_lin_vel_;
    double track_width_;

    // Total arc length of the segment; constant, so computed once.
    double total_length_;

    // Arc length as a function of the Bezier parameter, sampled on a uniform
    // grid in the parameter. This is both the grid the velocity ceiling is
    // sampled on and the s <-> t map used to place each pose. Built once because
    // it does not depend on the profile's state.
    std::vector<double> limit_t_;
    std::vector<double> limit_s_;

    // The 1D velocity profiler this class adapts.
    ScalarProfile scalar_;
    // Samples already turned into poses, so each step converts only the newest.
    size_t emitted_;

    // Accumulated output
    std::vector<Pose> poses_;
    std::vector<VelocityLayout> velocities_;
};
