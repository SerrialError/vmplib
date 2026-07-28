// ramsete.cpp
#include "ramsete.hpp"
#include <cmath>

using namespace MotionUtils;

RamseteFollower::RamseteFollower(const std::vector<Pose>& refPoses,
                                 const std::vector<VelocityLayout>& refVels,
				 float trackWidth,
                                 float bGain,
                                 float zetaGain,
				 float timeAccum,
                                 float dt,
				 bool reverse,
				 std::optional<Pose> initialPose)
    : track_width_(trackWidth),
      b_gain_(bGain),
      zeta_gain_(zetaGain),
      dt_(dt),
      reverse_(reverse),
      current_pose_{0.0f, 0.0f, 0.0f},
      time_accum_(timeAccum),
      index_(0),
      ref_poses_ptr_(&refPoses),
      ref_vels_ptr_(&refVels)
{
    executed_poses_.reserve(1000);
    executed_vels_.reserve(1000);
    current_pose_ = initialPose.value_or(refPoses.front());
    if (reverse_) {
        current_pose_.theta = wrapAngle(current_pose_.theta + static_cast<float>(M_PI));
    }
    executed_poses_.clear();
    executed_vels_.clear();
}

bool RamseteFollower::isFinished() const {
    return (ref_poses_ptr_ == nullptr) || (index_ >= ref_poses_ptr_->size());
}

Pose RamseteFollower::getCurrentPose() const {
    return current_pose_;
}

const std::vector<Pose>& RamseteFollower::getExecutedPoses() const {
    return executed_poses_;
}

const std::vector<VelocityLayout>& RamseteFollower::getExecutedVelocities() const {
    return executed_vels_;
}

float RamseteFollower::sinc(float x) {
    return (std::abs(x) < 1e-5f) ? 1.0f : std::sin(x) / x;
}

VelocityLayout RamseteFollower::step() {
    if (isFinished()) {
        return {0.0f, 0.0f, time_accum_};
    }

    // Grab reference at index_
    const Pose&    refPose = (*ref_poses_ptr_)[index_];
    const VelocityLayout& refV  = (*ref_vels_ptr_)[index_];

    // If reversing, flip sign of linear velocity and offset theta by π
    float v_ref = refV.linear;
    float w_ref = refV.angular;
    float theta_ref = refPose.theta;
    if (reverse_) {
        v_ref = -v_ref;
        theta_ref = wrapAngle(theta_ref + static_cast<float>(M_PI));
    }

    // Compute errors in robot frame
    float error_theta = wrapAngle(theta_ref - current_pose_.theta);
    float dx = refPose.x - current_pose_.x;
    float dy = refPose.y - current_pose_.y;
    float cos_t = std::cos(current_pose_.theta);
    float sin_t = std::sin(current_pose_.theta);
    float error_x =  sin_t * dy + cos_t * dx;
    float error_y =  cos_t * dy - sin_t * dx;

    // k = 2*zeta*sqrt(w_ref^2 + b*v_ref^2), with b in 1/m^2 so that k is 1/s.
    float k = 2.0f * zeta_gain_ * std::sqrt(w_ref * w_ref + b_gain_ * v_ref * v_ref);

    float v_real = v_ref * std::cos(error_theta) + k * error_x;
    float w_real = w_ref + k * error_theta
                 + b_gain_ * v_ref * sinc(error_theta) * error_y;

    // Advance time & pose
    time_accum_ += dt_;
    current_pose_.x += v_real * std::cos(current_pose_.theta) * dt_;
    current_pose_.y += v_real * std::sin(current_pose_.theta) * dt_;
    current_pose_.theta = wrapAngle(current_pose_.theta + w_real * dt_);

    // Log
    executed_poses_.push_back(current_pose_);
    executed_vels_.push_back({ v_real, w_real, time_accum_ });

    ++index_;
    return { v_real, w_real, time_accum_ };
}
