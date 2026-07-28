#include "motion-profiling.hpp"
#include <algorithm>
#include <cmath>

using namespace MotionUtils;

TrapezoidalProfile::TrapezoidalProfile(
    const std::vector<Point>& controlPts,
    float maxLinVel,
    float maxLinAccel,
    float timeAccum,
    float startVel,
    float endVel,
    const std::vector<KeyframeVelocities>& keyframes,
    bool useKeyframes,
    float dt
)
    : s_current_(0.0f),
      prev_t_(0.0f),
      time_accum_(timeAccum),
      cur_speed_(startVel),
      prev_keyframe_idx_(0),
      step_count_(0),
      control_(controlPts),
      max_lin_vel_(maxLinVel),
      max_lin_accel_(maxLinAccel),
      exit_velocity_(endVel),
      use_keyframes_(useKeyframes),
      dt_(dt),
      keyframes_(keyframes),
      total_length_(sFunction(controlPts, 1.0f)),
      max_steps_(static_cast<size_t>(60.0f / dt))
{
    poses_.reserve(1000);
    velocities_.reserve(1000);
}

bool TrapezoidalProfile::isFinished() const {
    return prev_t_ >= 1.0f || step_count_ >= max_steps_;
}

const std::vector<Pose>& TrapezoidalProfile::getPoses() const {
    return poses_;
}

const std::vector<VelocityLayout>& TrapezoidalProfile::getVelocities() const {
    return velocities_;
}

float TrapezoidalProfile::computeCurvatureVelocityLimit(float t) const {
    const float trackWidth = 0.288925f;
    float curv = unsignedCurvature(control_, t);
    if (std::abs(curv) < 1e-6f) {
        return max_lin_vel_;
    }
    float turn_radius = 1.0f / curv;
    return max_lin_vel_ * turn_radius / (turn_radius + trackWidth / 2.0f);
}

// Speed reachable in one timestep given the acceleration limit.
float TrapezoidalProfile::computeAccelerationLimit() const {
    return cur_speed_ + (max_lin_accel_ * dt_);
}

// Largest speed from which the robot can still reach exit_velocity_ by the end
// of the segment: v = sqrt(v_end^2 + 2*a*d_remaining). Beyond the braking zone
// this exceeds max_lin_vel_, so it drops out of the min on its own.
float TrapezoidalProfile::computeBrakingLimit(float s) const {
    float remaining = total_length_ - s;
    if (remaining <= 0.0f) {
        return exit_velocity_;
    }
    return std::sqrt(exit_velocity_ * exit_velocity_ + 2.0f * max_lin_accel_ * remaining);
}

float TrapezoidalProfile::computeKeyframeLimit() {
    if (!use_keyframes_ || prev_keyframe_idx_ + 1 >= keyframes_.size()) {
        return std::numeric_limits<float>::infinity();
    }

    const auto& kf0 = keyframes_[prev_keyframe_idx_];
    const auto& kf1 = keyframes_[prev_keyframe_idx_ + 1];

    if (kf1.time < time_accum_) {
        ++prev_keyframe_idx_;
    }

    float vi2 = kf0.velocity * kf0.velocity;
    float vf2 = kf1.velocity * kf1.velocity;
    float s_now = sFunction(control_, prev_t_);
    float s_kf1 = sFunction(control_, kf1.time);

    if (s_kf1 <= 0.0f) {
        return kf1.velocity;
    }
    float ratio = s_now / s_kf1;
    float vel_lim_sq = vi2 + (vf2 - vi2) * ratio;
    return (vel_lim_sq > 0.0f ? std::sqrt(vel_lim_sq) : kf1.velocity);
}

float TrapezoidalProfile::findNextT(float s0, float deltaS) const {
    return findTForS(control_, s0, deltaS);
}
void TrapezoidalProfile::start() {
    Pose newPose = findXandY(control_, 0.0f);
    poses_.push_back(newPose);
    VelocityLayout vlay{ 0.0f, 0.0f, time_accum_ };
    velocities_.push_back(vlay);
}

void TrapezoidalProfile::step() {
    ++step_count_;
    time_accum_ += dt_;
    s_current_ = sFunction(control_, prev_t_);

    float keyframe_lim  = computeKeyframeLimit();
    float curvature_lim = computeCurvatureVelocityLimit(prev_t_);
    float accel_lim     = computeAccelerationLimit();
    float brake_lim     = computeBrakingLimit(s_current_);

    // These bound how fast the speed may rise, but nothing bounds how fast it
    // may fall: a curvature spike still produces a step change that exceeds
    // max_lin_accel_. Fixing that needs a backward pass over the whole path,
    // not a per-step clamp. See the "acceleration limit" test.
    float desired_linear = std::min({ curvature_lim,
                                      accel_lim,
                                      brake_lim,
                                      keyframe_lim,
                                      max_lin_vel_ });
    float deltaS = desired_linear * dt_;
    float next_t = findNextT(s_current_, deltaS);

    float kappa = signedCurvature(control_, next_t);
    float turning_component = kappa * desired_linear;

    Pose newPose = findXandY(control_, next_t);
    poses_.push_back(newPose);

    VelocityLayout vlay{ desired_linear, turning_component, time_accum_ };
    velocities_.push_back(vlay);

    prev_t_    = next_t;
    cur_speed_ = desired_linear;           
}
