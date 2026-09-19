// ramsete.cpp
#include "ramsete.hpp"
#include "motion-utils.hpp"
#include <cmath>

using MotionUtils::wrapAngle;

namespace {
// Enough for a few seconds of tracking at a typical dt, so the common case
// never reallocates.
constexpr size_t kExpectedSamples = 1000;
} // namespace

RamseteFollower::RamseteFollower(const std::vector<Pose>& refPoses,
                                 const std::vector<VelocityLayout>& refVels,
                                 double trackWidth,
                                 double bGain,
                                 double zetaGain,
                                 double timeAccum,
                                 double dt,
                                 bool reverse,
                                 std::optional<Pose> initialPose)
    : track_width_(trackWidth),
      b_gain_(bGain),
      zeta_gain_(zetaGain),
      dt_(dt),
      reverse_(reverse),
      current_pose_{0.0, 0.0, 0.0},
      time_accum_(timeAccum),
      index_(0),
      ref_poses_ptr_(&refPoses),
      ref_vels_ptr_(&refVels)
{
    executed_poses_.reserve(kExpectedSamples);
    executed_vels_.reserve(kExpectedSamples);
    current_pose_ = initialPose.value_or(refPoses.front());
    if (reverse_) {
        current_pose_.theta = wrapAngle(current_pose_.theta + M_PI);
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

double RamseteFollower::sinc(double x) {
    return (std::abs(x) < 1e-9) ? 1.0 : std::sin(x) / x;
}

VelocityLayout RamseteFollower::step() {
    if (isFinished()) {
        return {0.0, 0.0, time_accum_};
    }

    // Grab reference at index_
    const Pose&    refPose = (*ref_poses_ptr_)[index_];
    const VelocityLayout& refV  = (*ref_vels_ptr_)[index_];

    // If reversing, flip sign of linear velocity and offset theta by π
    double v_ref = refV.linear;
    double w_ref = refV.angular;
    double theta_ref = refPose.theta;
    if (reverse_) {
        v_ref = -v_ref;
        theta_ref = wrapAngle(theta_ref + M_PI);
    }

    // Compute errors in robot frame
    double error_theta = wrapAngle(theta_ref - current_pose_.theta);
    double dx = refPose.x - current_pose_.x;
    double dy = refPose.y - current_pose_.y;
    double cos_t = std::cos(current_pose_.theta);
    double sin_t = std::sin(current_pose_.theta);
    double error_x =  sin_t * dy + cos_t * dx;
    double error_y =  cos_t * dy - sin_t * dx;

    // k = 2*zeta*sqrt(w_ref^2 + b*v_ref^2), with b in 1/m^2 so that k is 1/s.
    double k = 2.0 * zeta_gain_ * std::sqrt(w_ref * w_ref + b_gain_ * v_ref * v_ref);

    double v_real = v_ref * std::cos(error_theta) + k * error_x;
    double w_real = w_ref + k * error_theta
                 + b_gain_ * v_ref * sinc(error_theta) * error_y;

    // Log the pre-step pose. This sample is the robot's state *while* tracking
    // reference sample index_, so it is stamped with that reference's own time
    // (refV.time) and executed[i] lines up with planned[i] in both pose and
    // time -- for every segment, not just the first. Logging after integration
    // instead put executed[i] a whole step ahead, on planned[i+1], and stamped
    // it with planned[i+1]'s time.
    const VelocityLayout sample{ v_real, w_real, refV.time };
    executed_poses_.push_back(current_pose_);
    executed_vels_.push_back(sample);

    // Advance time & pose for the next step. Holding v and omega for a step
    // drives an arc, whose chord points along the heading halfway through the
    // turn and is sinc(turn / 2) as long as the distance driven. Stepping along
    // the starting heading instead drifts outward by about v * dt * turn / 2 per
    // step.
    time_accum_ += dt_;
    const double turn = w_real * dt_;
    const double chord = v_real * dt_ * sinc(turn / 2.0);
    const double heading = current_pose_.theta + turn / 2.0;
    current_pose_.x += chord * std::cos(heading);
    current_pose_.y += chord * std::sin(heading);
    current_pose_.theta = wrapAngle(current_pose_.theta + turn);

    ++index_;
    return sample;
}
