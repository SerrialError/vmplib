#include "motion-profiling.hpp"
#include "bezier.hpp"
#include <algorithm>
#include <cmath>

namespace {
// Samples used to build the velocity limit curve. The grid is uniform in the
// Bezier parameter, so its arc-length spacing follows the parametric speed.
// 256 keeps that spacing well under one timestep of travel for the path lengths
// and speeds this library targets.
constexpr int kLimitSamples = 256;

// Enough for a few seconds of travel at a typical dt, so the common case never
// reallocates.
constexpr size_t kExpectedSamples = 1000;

// Curvature velocity ceiling: the outer wheel of a differential drive travels a
// wider arc than the path centre, so a tight turn caps centre speed.
double curvatureVelocityLimit(const std::vector<Point>& control, double t,
                              double maxLinVel, double trackWidth) {
    const double curv = unsignedCurvature(control, t);
    if (std::abs(curv) < 1e-9) {
        return maxLinVel;
    }
    const double turn_radius = 1.0 / curv;
    return maxLinVel * turn_radius / (turn_radius + trackWidth / 2.0);
}

// Build the arc-length table and the curvature-limited velocity ceiling on a
// grid uniform in the Bezier parameter. Arc length accumulates one quadrature
// panel per cell, so the whole s(t) table costs a single sweep rather than
// integrating from zero at every sample.
void buildArcLengthTable(const std::vector<Point>& control, double maxLinVel,
                         double trackWidth, std::vector<double>& outT,
                         std::vector<double>& outS, std::vector<double>& outCeiling) {
    outT.resize(kLimitSamples);
    outS.resize(kLimitSamples);
    outCeiling.resize(kLimitSamples);
    for (int i = 0; i < kLimitSamples; i++) {
        const double t = static_cast<double>(i) / (kLimitSamples - 1);
        outT[i] = t;
        outS[i] = (i == 0) ? 0.0 : outS[i - 1] + arcLength(control, outT[i - 1], t);
        outCeiling[i] = std::min(maxLinVel,
                                 curvatureVelocityLimit(control, t, maxLinVel, trackWidth));
    }
}
} // namespace

BezierPathProfile::BezierPathProfile(
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
    double startArcLength
)
    : control_(controlPts),
      max_lin_vel_(maxLinVel),
      track_width_(trackWidth),
      total_length_(0.0),
      scalar_(
          // A valid one-node placeholder. The real profile needs the arc-length
          // table built in the body below, so it is move-assigned into scalar_
          // once that exists.
          0.0, std::vector<double>{ 0.0 }, std::vector<double>{ maxLinVel },
          maxLinAccel, startVel, endVel, dt, {}),
      emitted_(0)
{
    // The profiler reaches into control_[0..3] throughout; refuse a malformed
    // segment here rather than let a bezier* routine read past its end.
    requireCubicSegment(control_);

    std::vector<double> ceiling;
    buildArcLengthTable(control_, max_lin_vel_, track_width_, limit_t_, limit_s_, ceiling);
    total_length_ = limit_s_.back();

    // Keyframes are pinned to a Bezier parameter; the scalar core is indexed by
    // arc length, so project each onto the arc-length table first. When
    // keyframes are disabled the core simply sees none.
    std::vector<ScalarKeyframe> scalarKeyframes;
    if (useKeyframes) {
        scalarKeyframes.reserve(keyframes.size());
        for (const auto& kf : keyframes) {
            scalarKeyframes.push_back(ScalarKeyframe{ kf.velocity, arcLengthAt(kf.t) });
        }
    }

    scalar_ = ScalarProfile(total_length_, limit_s_, std::move(ceiling), maxLinAccel,
                            startVel, endVel, dt, std::move(scalarKeyframes), startArcLength,
                            timeAccum);

    poses_.reserve(kExpectedSamples);
    velocities_.reserve(kExpectedSamples);
}

double BezierPathProfile::overshootArcLength() const {
    return scalar_.overshootDistance();
}

bool BezierPathProfile::isFinished() const {
    return scalar_.isFinished();
}

const std::vector<Pose>& BezierPathProfile::getPoses() const {
    return poses_;
}

const std::vector<VelocityLayout>& BezierPathProfile::getVelocities() const {
    return velocities_;
}

double BezierPathProfile::arcLengthAt(double t) const {
    if (t <= 0.0) {
        return 0.0;
    }
    if (t >= 1.0) {
        return total_length_;
    }
    // The grid is uniform in t, so the cell index is arithmetic, not a search.
    const double scaled = t * (kLimitSamples - 1);
    const int lo = std::min(static_cast<int>(scaled), kLimitSamples - 2);
    const double frac = scaled - static_cast<double>(lo);
    return limit_s_[lo] + (limit_s_[lo + 1] - limit_s_[lo]) * frac;
}

double BezierPathProfile::parameterAt(double s) const {
    if (s <= 0.0) {
        return 0.0;
    }
    if (s >= total_length_) {
        return 1.0;
    }
    const auto it = std::upper_bound(limit_s_.begin(), limit_s_.end(), s);
    const size_t hi = static_cast<size_t>(it - limit_s_.begin());
    const size_t lo = hi - 1;
    const double span = limit_s_[hi] - limit_s_[lo];
    if (span <= 0.0) {
        return limit_t_[hi];
    }
    return limit_t_[lo] + (limit_t_[hi] - limit_t_[lo]) * (s - limit_s_[lo]) / span;
}

void BezierPathProfile::emitSamples() {
    const auto& samples = scalar_.samples();
    for (; emitted_ < samples.size(); ++emitted_) {
        const ScalarSample& s = samples[emitted_];
        const double t = parameterAt(s.position);
        const double kappa = signedCurvature(control_, t);
        poses_.push_back(findXandY(control_, t));
        velocities_.push_back(VelocityLayout{ s.velocity, kappa * s.velocity, s.time });
    }
}

void BezierPathProfile::start() {
    scalar_.start();
    emitSamples();
}

void BezierPathProfile::step() {
    scalar_.step();
    emitSamples();
}
