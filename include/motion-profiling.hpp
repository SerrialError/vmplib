#pragma once

#include <vector>
#include "types.hpp"
#include "motion-utils.hpp"

class TrapezoidalProfile {
public:
    TrapezoidalProfile(
        const std::vector<Point>& controlPts,
        float maxLinVel,
        float maxLinAccel,
        float trackWidth,
        float timeAccum,
        float startVel,
        float endVel,
        const std::vector<KeyframeVelocities>& keyframes,
        bool useKeyframes,
        float dt
    );

    // Advance one timestep. Returns (linear, angular, time)
    void start();
    void step();

    // True once t ≥ 1.0
    bool isFinished() const;

    // Access generated path poses & velocities
    const std::vector<Pose>& getPoses() const;
    const std::vector<VelocityLayout>& getVelocities() const;
    

private:
    // Internal state
    float s_current_;
    float prev_t_;
    float time_accum_;
    float cur_speed_;
    size_t prev_keyframe_idx_;
    size_t step_count_;

    // Parameters
    const std::vector<Point>& control_;
    float max_lin_vel_;
    float max_lin_accel_;
    float track_width_;
    float exit_velocity_;
    bool use_keyframes_;
    float dt_;
    std::vector<KeyframeVelocities> keyframes_;

    // Total arc length of the segment; constant, so computed once.
    float total_length_;
    // Backstop so a non-advancing profile fails fast instead of looping forever.
    size_t max_steps_;

    // Accumulated output
    std::vector<Pose> poses_;
    std::vector<VelocityLayout> velocities_;

    // Helper methods
    float computeCurvatureVelocityLimit(float t) const;
    float computeAccelerationLimit() const;
    float computeBrakingLimit(float s) const;
    float computeKeyframeLimit();
    float findNextT(float s0, float deltaS) const;
};
