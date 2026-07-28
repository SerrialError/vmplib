// ramsete.hpp
#pragma once

#include <optional>
#include <vector>
#include "types.hpp"
#include "motion-utils.hpp"

class RamseteFollower {
public:
    // bGain > 0 and zetaGain in (0, 1) per the RAMSETE stability conditions.
    // initialPose defaults to the first reference pose; pass a different one to
    // start the robot off the path.
    RamseteFollower(const std::vector<Pose>& refPoses,
                    const std::vector<VelocityLayout>& refVels,
		    float trackWidth,
                    float bGain,
                    float zetaGain,
		    float timeAccum,
                    float dt,
		    bool reverse,
		    std::optional<Pose> initialPose = std::nullopt);

    // Advance one timestep. Returns the new robot Pose & velocity.
    VelocityLayout step();

    // True when we have walked through all reference points
    bool isFinished() const;

    // Access the executed path and velocities
    const std::vector<Pose>& getExecutedPoses() const;
    const std::vector<VelocityLayout>& getExecutedVelocities() const;

    Pose getCurrentPose() const;

private:
    float track_width_;
    float b_gain_;
    float zeta_gain_;
    float dt_;
    bool  reverse_;

    // Internal state
    Pose  current_pose_;    // robot’s current pose
    float time_accum_;
    size_t index_;

    // References (set in initialize)
    const std::vector<Pose>*           ref_poses_ptr_;
    const std::vector<VelocityLayout>* ref_vels_ptr_;

    // Logged trajectory
    std::vector<Pose> executed_poses_;
    std::vector<VelocityLayout> executed_vels_;

    // Private helpers
    static float sinc(float x);
};
