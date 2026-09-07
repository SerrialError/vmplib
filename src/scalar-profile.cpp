#include "scalar-profile.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
// A profile that stops advancing would otherwise loop forever. No single path
// this library is meant to plan takes anywhere near a minute to drive.
constexpr double kWatchdogSeconds = 60.0;

// Enough for a few seconds of travel at a typical dt, so the common case never
// reallocates.
constexpr size_t kExpectedSamples = 1000;
} // namespace

ScalarProfile::ScalarProfile(
    double distance,
    std::vector<double> ceilingDistances,
    std::vector<double> ceilingVelocities,
    double maxAccel,
    double startVel,
    double endVel,
    double dt,
    std::vector<ScalarKeyframe> keyframes,
    double startDistance,
    double timeAccum
)
    : distance_(distance),
      max_accel_(maxAccel),
      exit_velocity_(endVel),
      dt_(dt),
      limit_s_(std::move(ceilingDistances)),
      s_current_(std::min(std::max(0.0, startDistance), distance)),
      cur_speed_(startVel),
      time_accum_(timeAccum),
      // A carry-over longer than the whole path skips it outright and has to
      // keep travelling into whatever comes next.
      overshoot_(std::max(0.0, startDistance - distance)),
      step_count_(0),
      max_steps_(static_cast<size_t>(kWatchdogSeconds / dt))
{
    buildVelocityLimits(ceilingVelocities, keyframes);

    // A path too short to brake in cannot honour the requested start velocity;
    // the backward pass has already worked out what it can do.
    cur_speed_ = std::min(cur_speed_, velocityAt(s_current_));

    samples_.reserve(kExpectedSamples);
}

double ScalarProfile::overshootDistance() const {
    return overshoot_;
}

bool ScalarProfile::isFinished() const {
    return s_current_ >= distance_ || step_count_ >= max_steps_;
}

const std::vector<ScalarSample>& ScalarProfile::samples() const {
    return samples_;
}

// Speed reachable in one timestep given the acceleration limit.
double ScalarProfile::accelerationLimit() const {
    return cur_speed_ + (max_accel_ * dt_);
}

double ScalarProfile::keyframeCeiling(double s, const std::vector<ScalarKeyframe>& keyframes,
                                      size_t& idx) const {
    if (keyframes.size() < 2) {
        return std::numeric_limits<double>::infinity();
    }

    while (idx + 2 < keyframes.size() && s >= keyframes[idx + 1].distance) {
        ++idx;
    }

    const double s0 = keyframes[idx].distance;
    const double s1 = keyframes[idx + 1].distance;
    const double v0 = keyframes[idx].velocity;
    const double v1 = keyframes[idx + 1].velocity;

    const double span = s1 - s0;
    if (span <= 0.0) {
        return v1;
    }

    // Interpolate in v^2, which makes each keyframe interval a constant
    // acceleration segment: v^2 = v0^2 + 2*a*(s - s0).
    const double lambda = std::clamp((s - s0) / span, 0.0, 1.0);
    const double vsq = v0 * v0 + (v1 * v1 - v0 * v0) * lambda;
    return vsq > 0.0 ? std::sqrt(vsq) : 0.0;
}

// g(v) = v^2 + a*dt*v. Braking at the limit advances g by exactly 2*a*ds per
// unit of arc length covered, so the ramp is a straight line in g.
double ScalarProfile::brakingPotential(double v) const {
    return v * v + max_accel_ * dt_ * v;
}

double ScalarProfile::velocityAtPotential(double g) const {
    const double adt = max_accel_ * dt_;
    return 0.5 * (std::sqrt(adt * adt + 4.0 * std::max(0.0, g)) - adt);
}

void ScalarProfile::buildVelocityLimits(const std::vector<double>& ceilingVelocities,
                                        const std::vector<ScalarKeyframe>& keyframes) {
    const size_t n = limit_s_.size();
    limit_v_.resize(n);

    // Hard ceilings first: the per-node ceiling the caller supplied (top speed,
    // plus the curvature limit for a 2D path) and the user's keyframes. Neither
    // knows anything about what the drivetrain can reach.
    size_t kfIdx = 0;
    for (size_t i = 0; i < n; i++) {
        limit_v_[i] = std::min(ceilingVelocities[i],
                               keyframeCeiling(limit_s_[i], keyframes, kfIdx));
    }

    // Backward pass, swept from the end. This is what makes the profile slow
    // down *before* a constrained region instead of stepping down at it, and it
    // subsumes the old end-of-path braking limit. Only deceleration needs
    // precomputing, because only deceleration depends on what lies ahead; the
    // matching forward pass is applied online in step() as the per-timestep
    // acceleration cap.
    //
    // The textbook ramp v^2 = v_next^2 + 2*a*ds is the continuous one, and a
    // fixed timestep cannot ride it down: each step covers v*dt, so following
    // it drops v by v - sqrt(v^2 - 2*a*v*dt), which grows to the whole of v as
    // v approaches 2*a*dt. The tail therefore lands short of the exit velocity
    // and has to give the remainder back in one step, well past the
    // acceleration limit.
    //
    // Stepping the recurrence backwards instead (v_j = v_end + j*a*dt at
    // r_j = r_(j-1) + v_j*dt) gives a ramp that costs half a step's travel more
    // and is linear in g(v) = v^2 + a*dt*v. Riding it down spends exactly
    // a*dt per step and lands on the exit velocity, leaving a residual of at
    // most a*dt for the endpoint sample in step() to absorb.
    limit_v_.back() = std::min(limit_v_.back(), exit_velocity_);
    for (int i = static_cast<int>(n) - 2; i >= 0; i--) {
        const double ds = limit_s_[i + 1] - limit_s_[i];
        const double reachable = velocityAtPotential(brakingPotential(limit_v_[i + 1]) +
                                                     2.0 * max_accel_ * ds);
        limit_v_[i] = std::min(limit_v_[i], reachable);
    }
}

double ScalarProfile::velocityAt(double s) const {
    if (s <= 0.0) {
        return limit_v_.front();
    }
    if (s >= distance_) {
        return limit_v_.back();
    }
    const auto it = std::upper_bound(limit_s_.begin(), limit_s_.end(), s);
    const size_t hi = static_cast<size_t>(it - limit_s_.begin());
    const size_t lo = hi - 1;
    const double span = limit_s_[hi] - limit_s_[lo];
    if (span <= 0.0) {
        return limit_v_[hi];
    }
    // Interpolate in the braking potential, the metric the backward pass is
    // linear in. Interpolating in v^2 instead would restore the continuous
    // ramp's shape inside the final cell, which is where the profile is least
    // able to afford it: v^2 leaves the curve going as sqrt(distance-to-go)
    // near the end, where the feasible ramp is very nearly linear in it.
    const double lambda = (s - limit_s_[lo]) / span;
    const double g0 = brakingPotential(limit_v_[lo]);
    const double g1 = brakingPotential(limit_v_[hi]);
    return velocityAtPotential(g0 + (g1 - g0) * lambda);
}

void ScalarProfile::start() {
    samples_.push_back(ScalarSample{ s_current_, cur_speed_, 0.0, time_accum_ });
}

void ScalarProfile::step() {
    ++step_count_;
    time_accum_ += dt_;

    // The robot crosses the step at the speed it is holding where the step
    // begins. The limit curve caps deceleration; the acceleration cap is the
    // forward pass, applied here one timestep at a time.
    const double travel_speed = std::min(velocityAt(s_current_), accelerationLimit());

    // The step that ends the path lands past the end. Record by how much so a
    // caller stitching the next profile on can start there instead of
    // discarding the travel.
    const double s_target = s_current_ + travel_speed * dt_;
    overshoot_ = std::max(0.0, s_target - distance_);
    const double landed = std::min(s_target, distance_);

    // A sample states the speed to hold *at* the position it carries, so the
    // limit curve is read where the step lands rather than where it started.
    // Reading it at the start put each sample half a step out of phase with its
    // own position: harmless in the middle of a path, but the last sample lands
    // on the endpoint, where the curve has already fallen to the exit velocity
    // while the departing speed has not. That is why a path planned to stop used
    // to trail off at a tenth of a metre per second instead of at rest.
    const double next_speed = std::min(velocityAt(s_target), accelerationLimit());
    const double accel = (next_speed - cur_speed_) / dt_;

    samples_.push_back(ScalarSample{ landed, next_speed, accel, time_accum_ });

    s_current_ = landed;
    cur_speed_ = next_speed;
}
