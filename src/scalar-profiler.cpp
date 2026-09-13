#include "scalar-profiler.hpp"
#include "config-error.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace {
// The velocity ceiling is sampled on a grid this many nodes fine at most, so a
// very long move does not allocate without bound.
constexpr int kMaxCeilingNodes = 8192;

// Throws MoveError unless speed is non-negative and finite. Direction comes from
// the sign of the distance, so a negative speed has no meaning.
void requireSpeed(const std::string& name, double speed) {
    if (!(speed >= 0.0) || !std::isfinite(speed)) {
        std::ostringstream msg;
        msg << name << " must be a non-negative, finite speed; got " << speed;
        throw MoveError(msg.str());
    }
}

// ScalarProfile works in magnitudes along the direction of travel; this puts
// the sign back. Zero stays +0 so a reverse move does not print "-0".
double signedValue(double direction, double magnitude) {
    return magnitude == 0.0 ? 0.0 : direction * magnitude;
}

// Runs profile to completion and appends its samples to out, placed on the axis
// at origin + direction * (distance travelled). emitStart is false for a
// profile that continues from out's last sample, which already covers its
// starting point.
void appendSamples(ScalarProfile& profile, bool emitStart, double origin, double direction,
                   std::vector<ScalarSample>& out) {
    if (emitStart) {
        profile.start();
    }
    while (!profile.isFinished()) {
        profile.step();
    }
    for (const ScalarSample& s : profile.samples()) {
        out.push_back(ScalarSample{ origin + direction * s.position,
                                    signedValue(direction, s.velocity),
                                    signedValue(direction, s.accel), s.time });
    }
}
} // namespace

std::vector<ScalarSample> generateScalarProfile(
    double distance,
    const ScalarProfileConfig& config,
    double startVel,
    double endVel,
    std::vector<ScalarKeyframe> keyframes
) {
    const double maxVelocity = requireLimit("maxVelocity", config.maxVelocity);
    const double maxAccel = requireLimit("maxAccel", config.maxAccel);
    requirePositiveLimit("dt", config.dt);
    requireSpeed("startVel", startVel);
    requireSpeed("endVel", endVel);
    for (const ScalarKeyframe& kf : keyframes) {
        requireSpeed("keyframe velocity", kf.velocity);
    }
    if (!std::isfinite(distance)) {
        throw MoveError("distance must be finite");
    }

    if (distance == 0.0) {
        return {};
    }
    const double direction = distance < 0.0 ? -1.0 : 1.0;
    const double length = std::fabs(distance);

    // One grid node roughly every top-speed timestep of travel, so the velocity
    // ceiling is sampled at least as finely as the output the profiler emits.
    const double spacing = std::max(maxVelocity * config.dt, 1e-6);
    int nodes = static_cast<int>(std::ceil(length / spacing)) + 1;
    nodes = std::clamp(nodes, 2, kMaxCeilingNodes);

    std::vector<double> ceilingDistances(nodes);
    for (int i = 0; i < nodes; ++i) {
        ceilingDistances[i] = length * static_cast<double>(i) / (nodes - 1);
    }
    // Flat ceiling: with no path there is no curvature to cap speed, only the
    // top speed. Keyframes are folded in by the profiler.
    std::vector<double> ceilingVelocities(nodes, maxVelocity);

    // ScalarProfile sweeps keyframes in order, so they must be sorted by
    // distance.
    std::sort(keyframes.begin(), keyframes.end(),
              [](const ScalarKeyframe& a, const ScalarKeyframe& b) {
                  return a.distance < b.distance;
              });

    ScalarProfile profile(length, std::move(ceilingDistances), std::move(ceilingVelocities),
                          maxAccel, startVel, endVel, config.dt, std::move(keyframes));
    std::vector<ScalarSample> samples;
    appendSamples(profile, true, 0.0, direction, samples);
    return samples;
}
