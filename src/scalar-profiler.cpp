#include "scalar-profiler.hpp"
#include "config-error.hpp"

#include <algorithm>
#include <cmath>

namespace {
// The velocity ceiling is sampled on a grid this many nodes fine at most, so a
// very long move does not allocate without bound.
constexpr int kMaxCeilingNodes = 8192;
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

    if (distance <= 0.0) {
        return {};
    }

    // One grid node roughly every top-speed timestep of travel, so the velocity
    // ceiling is sampled at least as finely as the output the profiler emits.
    const double spacing = std::max(maxVelocity * config.dt, 1e-6);
    int nodes = static_cast<int>(std::ceil(distance / spacing)) + 1;
    nodes = std::clamp(nodes, 2, kMaxCeilingNodes);

    std::vector<double> ceilingDistances(nodes);
    for (int i = 0; i < nodes; ++i) {
        ceilingDistances[i] = distance * static_cast<double>(i) / (nodes - 1);
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

    ScalarProfile profile(distance, std::move(ceilingDistances), std::move(ceilingVelocities),
                          maxAccel, startVel, endVel, config.dt, std::move(keyframes));
    profile.start();
    while (!profile.isFinished()) {
        profile.step();
    }
    return profile.samples();
}
