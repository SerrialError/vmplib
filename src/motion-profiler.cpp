#include "motion-profiler.hpp"
#include "motion-profiling.hpp"   // TrapezoidalProfile
#include "ramsete.hpp"            // RamseteFollower
#include <algorithm>
#include <vector>

namespace {
// A keyframe within this much of t = 0 counts as sitting on the segment join,
// where the only room to brake for it is in the segment before.
constexpr float kJunctionKeyframeTolerance = 1e-3f;
} // namespace

Trajectory generateTrajectory(
    const std::vector<std::vector<Point>>& controlPoints,
    const std::vector<std::vector<KeyframeVelocitiesXandY>>& keyframeList,
    bool useKeyframes,
    const ProfileConfig& config
) {
    Trajectory out;
    float timeAccum = 0.0f;

    // Convert every segment's (x,y,velocity) keyframes to (velocity, t) up
    // front. A segment's exit velocity depends on the *next* segment's
    // keyframes, which cannot be resolved one segment at a time.
    std::vector<std::vector<KeyframeVelocities>> keyframesPerSegment(controlPoints.size());
    if (useKeyframes) {
        const size_t n = std::min(controlPoints.size(), keyframeList.size());
        for (size_t i = 0; i < n; ++i) {
            keyframesPerSegment[i] = convertToTFrame(controlPoints[i], keyframeList[i]);
        }
    }

    float carryOverArcLength = 0.0f;
    bool havePreviousSegment = false;

    for (size_t i = 0; i < controlPoints.size(); ++i) {
        const std::vector<KeyframeVelocities>& keyframes = keyframesPerSegment[i];

        // Velocity is continuous across a junction, so a segment starts at
        // whatever the previous one actually ended at. Only the first segment
        // is free to take its start velocity from a keyframe.
        float initialVel = 0.0f;
        if (havePreviousSegment) {
            initialVel = out.velocities.back().back().linear;
        } else if (!keyframes.empty()) {
            initialVel = keyframes.front().velocity;
        }

        float exitVel = keyframes.empty() ? 0.0f : keyframes.back().velocity;

        // A keyframe sitting on the first point of the next segment has no room
        // to be met inside that segment, so it is really a constraint on this
        // segment's exit velocity. Braking for it has to start here.
        if (i + 1 < keyframesPerSegment.size()) {
            const std::vector<KeyframeVelocities>& next = keyframesPerSegment[i + 1];
            if (!next.empty() && next.front().t <= kJunctionKeyframeTolerance) {
                exitVel = next.front().velocity;
            }
        }

        // Deceleration comes from the backward pass inside the profile, so no
        // braking distance is needed here. carryOverArcLength resumes where the
        // previous segment's last timestep overshot the join.
        TrapezoidalProfile profiler(
            controlPoints[i],
            config.maxVelocity,
            config.maxAccel,
            config.trackWidth,
            timeAccum,
            initialVel,
            exitVel,
            keyframes,
            useKeyframes,
            config.dt,
            carryOverArcLength
        );

        // Only the first segment emits a sample at its start pose. For the rest
        // that pose is the join, already covered by the previous segment's
        // final sample, and re-emitting it would duplicate a timestamp.
        if (!havePreviousSegment) {
            profiler.start();
        }
        while (!profiler.isFinished()) {
            profiler.step();
        }

        carryOverArcLength = profiler.overshootArcLength();
        if (profiler.getVelocities().empty()) {
            // The carry-over covered this segment whole, so it produced no
            // samples. The remainder rolls on to the next one.
            continue;
        }
        havePreviousSegment = true;
        out.poses.push_back(profiler.getPoses());
        out.velocities.push_back(profiler.getVelocities());

        RamseteFollower follower(
            profiler.getPoses(),
            profiler.getVelocities(),
            config.trackWidth,
            config.ramseteB,
            config.ramseteZeta,
            timeAccum,
            config.dt,
            false
        );
        while (!follower.isFinished()) {
            follower.step();
        }
        out.followedPoses.push_back(follower.getExecutedPoses());
        out.followedVelocities.push_back(follower.getExecutedVelocities());

        timeAccum = profiler.getVelocities().back().time;
    }

    return out;
}
