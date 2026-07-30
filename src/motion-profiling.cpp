#include "motion-profiling.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

using namespace MotionUtils;

namespace {
// Samples used to build the velocity limit curve. The grid is uniform in the
// Bezier parameter, so its arc-length spacing follows the parametric speed.
// 256 keeps that spacing well under one timestep of travel for the path lengths
// and speeds this library targets.
constexpr int kLimitSamples = 256;
} // namespace

TrapezoidalProfile::TrapezoidalProfile(
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
)
    : s_current_(0.0f),
      prev_t_(0.0f),
      time_accum_(timeAccum),
      cur_speed_(startVel),
      step_count_(0),
      control_(controlPts),
      max_lin_vel_(maxLinVel),
      max_lin_accel_(maxLinAccel),
      track_width_(trackWidth),
      exit_velocity_(endVel),
      use_keyframes_(useKeyframes),
      dt_(dt),
      keyframes_(keyframes),
      total_length_(0.0f),
      max_steps_(static_cast<size_t>(60.0f / dt))
{
    buildVelocityLimits();
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
    float curv = unsignedCurvature(control_, t);
    if (std::abs(curv) < 1e-6f) {
        return max_lin_vel_;
    }
    float turn_radius = 1.0f / curv;
    return max_lin_vel_ * turn_radius / (turn_radius + track_width_ / 2.0f);
}

// Speed reachable in one timestep given the acceleration limit.
float TrapezoidalProfile::computeAccelerationLimit() const {
    return cur_speed_ + (max_lin_accel_ * dt_);
}

// Velocity cap imposed by the keyframes bracketing s. keyframeS holds the arc
// length of each keyframe; idx is carried across calls so a monotonic sweep
// does not rescan the list.
float TrapezoidalProfile::keyframeCeiling(float s, const std::vector<float>& keyframeS,
                                          size_t& idx) const {
    if (!use_keyframes_ || keyframes_.size() < 2) {
        return std::numeric_limits<float>::infinity();
    }

    while (idx + 2 < keyframes_.size() && s >= keyframeS[idx + 1]) {
        ++idx;
    }

    const float s0 = keyframeS[idx];
    const float s1 = keyframeS[idx + 1];
    const float v0 = keyframes_[idx].velocity;
    const float v1 = keyframes_[idx + 1].velocity;

    const float span = s1 - s0;
    if (span <= 0.0f) {
        return v1;
    }

    // Interpolate in v^2, which makes each keyframe interval a constant
    // acceleration segment: v^2 = v0^2 + 2*a*(s - s0).
    const float lambda = std::clamp((s - s0) / span, 0.0f, 1.0f);
    const float vsq = v0 * v0 + (v1 * v1 - v0 * v0) * lambda;
    return vsq > 0.0f ? std::sqrt(vsq) : 0.0f;
}

void TrapezoidalProfile::buildVelocityLimits() {
    limit_t_.resize(kLimitSamples);
    limit_s_.resize(kLimitSamples);
    limit_v_.resize(kLimitSamples);

    // Arc length accumulates one quadrature panel per cell, so the whole s(t)
    // table costs a single sweep rather than integrating from zero at every
    // sample. It is also finer than sFunction's fixed panel count, so it is the
    // metric the rest of the profiler uses.
    for (int i = 0; i < kLimitSamples; i++) {
        const float t = static_cast<float>(i) / (kLimitSamples - 1);
        limit_t_[i] = t;
        limit_s_[i] = (i == 0) ? 0.0f
                               : limit_s_[i - 1] + arcLength(control_, limit_t_[i - 1], t);
    }
    total_length_ = limit_s_.back();

    // Hard ceilings first: geometry and the user's keyframes, neither of which
    // knows anything about what the drivetrain can reach.
    std::vector<float> keyframeS;
    keyframeS.reserve(keyframes_.size());
    for (const auto& kf : keyframes_) {
        keyframeS.push_back(arcLengthAt(kf.t));
    }

    size_t kfIdx = 0;
    for (int i = 0; i < kLimitSamples; i++) {
        limit_v_[i] = std::min({ max_lin_vel_,
                                 computeCurvatureVelocityLimit(limit_t_[i]),
                                 keyframeCeiling(limit_s_[i], keyframeS, kfIdx) });
    }

    // Backward pass: v^2 = v_next^2 + 2*a*ds, swept from the end. This is what
    // makes the profile slow down *before* a constrained region instead of
    // stepping down at it, and it subsumes the old end-of-segment braking
    // limit. Only deceleration needs precomputing, because only deceleration
    // depends on what lies ahead; the matching forward pass is applied online
    // in step() as the per-timestep acceleration cap.
    limit_v_.back() = std::min(limit_v_.back(), exit_velocity_);
    for (int i = kLimitSamples - 2; i >= 0; i--) {
        const float ds = limit_s_[i + 1] - limit_s_[i];
        const float reachable =
            std::sqrt(limit_v_[i + 1] * limit_v_[i + 1] + 2.0f * max_lin_accel_ * ds);
        limit_v_[i] = std::min(limit_v_[i], reachable);
    }

    // A segment too short to brake in cannot honour the requested start
    // velocity; the backward pass has already worked out what it can do.
    cur_speed_ = std::min(cur_speed_, limit_v_.front());
}

float TrapezoidalProfile::arcLengthAt(float t) const {
    if (t <= 0.0f) {
        return 0.0f;
    }
    if (t >= 1.0f) {
        return total_length_;
    }
    // The grid is uniform in t, so the cell index is arithmetic, not a search.
    const float scaled = t * (kLimitSamples - 1);
    const int lo = std::min(static_cast<int>(scaled), kLimitSamples - 2);
    const float frac = scaled - static_cast<float>(lo);
    return limit_s_[lo] + (limit_s_[lo + 1] - limit_s_[lo]) * frac;
}

float TrapezoidalProfile::parameterAt(float s) const {
    if (s <= 0.0f) {
        return 0.0f;
    }
    if (s >= total_length_) {
        return 1.0f;
    }
    const auto it = std::upper_bound(limit_s_.begin(), limit_s_.end(), s);
    const size_t hi = static_cast<size_t>(it - limit_s_.begin());
    const size_t lo = hi - 1;
    const float span = limit_s_[hi] - limit_s_[lo];
    if (span <= 0.0f) {
        return limit_t_[hi];
    }
    return limit_t_[lo] + (limit_t_[hi] - limit_t_[lo]) * (s - limit_s_[lo]) / span;
}

float TrapezoidalProfile::velocityAt(float s) const {
    if (s <= 0.0f) {
        return limit_v_.front();
    }
    if (s >= total_length_) {
        return limit_v_.back();
    }
    const auto it = std::upper_bound(limit_s_.begin(), limit_s_.end(), s);
    const size_t hi = static_cast<size_t>(it - limit_s_.begin());
    const size_t lo = hi - 1;
    const float span = limit_s_[hi] - limit_s_[lo];
    if (span <= 0.0f) {
        return limit_v_[hi];
    }
    // Interpolate in v^2: within a cell the profile is a constant-acceleration
    // ramp, which is linear in v^2 and not in v.
    const float lambda = (s - limit_s_[lo]) / span;
    const float v0sq = limit_v_[lo] * limit_v_[lo];
    const float v1sq = limit_v_[hi] * limit_v_[hi];
    const float vsq = v0sq + (v1sq - v0sq) * lambda;
    return vsq > 0.0f ? std::sqrt(vsq) : 0.0f;
}

void TrapezoidalProfile::start() {
    poses_.push_back(findXandY(control_, 0.0f));
    velocities_.push_back(
        VelocityLayout{ cur_speed_, signedCurvature(control_, 0.0f) * cur_speed_, time_accum_ });
}

void TrapezoidalProfile::step() {
    ++step_count_;
    time_accum_ += dt_;
    s_current_ = arcLengthAt(prev_t_);

    // The curve caps deceleration; the acceleration cap is the forward pass,
    // applied here one timestep at a time.
    const float desired_linear = std::min(velocityAt(s_current_), computeAccelerationLimit());
    const float next_t = parameterAt(s_current_ + desired_linear * dt_);

    const float kappa = signedCurvature(control_, next_t);
    poses_.push_back(findXandY(control_, next_t));
    velocities_.push_back(
        VelocityLayout{ desired_linear, kappa * desired_linear, time_accum_ });

    prev_t_ = next_t;
    cur_speed_ = desired_linear;
}
