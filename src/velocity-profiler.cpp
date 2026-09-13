#include "velocity-profiler.hpp"
#include "config-error.hpp"

#include <cmath>
#include <sstream>

namespace {
// A step count that should be whole computes as that number plus rounding noise
// (0.07 / 0.01 is 7.000000000000001), which a plain ceil would round up to a
// whole extra step.
constexpr double kWholeStepTolerance = 1e-9;

// Timesteps needed to cover amount in steps of size step, counting a final
// partial step.
long stepsToCover(double amount, double step) {
    return static_cast<long>(std::ceil(amount / step - kWholeStepTolerance));
}
} // namespace

std::vector<VelocitySample> generateVelocityProfile(
    const VelocityProfileConfig& config,
    const std::vector<VelocityTarget>& targets,
    double startVelocity
) {
    const double maxAccel = requireLimit("maxAccel", config.maxAccel);
    if (config.maxVelocity) {
        requirePositiveLimit("maxVelocity", *config.maxVelocity);
    }
    requirePositiveLimit("dt", config.dt);

    const auto checkVelocity = [&](const std::string& name, double velocity) {
        if (!std::isfinite(velocity)) {
            throw VelocityTargetError(name + " velocity must be finite");
        }
        if (config.maxVelocity && std::fabs(velocity) > *config.maxVelocity) {
            std::ostringstream msg;
            msg << name << " velocity " << velocity << " is faster than maxVelocity "
                << *config.maxVelocity;
            throw VelocityTargetError(msg.str());
        }
    };
    checkVelocity("start", startVelocity);
    for (size_t i = 0; i < targets.size(); ++i) {
        const std::string name = "target " + std::to_string(i + 1);
        checkVelocity(name, targets[i].velocity);
        if (!(targets[i].hold >= 0.0) || !std::isfinite(targets[i].hold)) {
            throw VelocityTargetError(name + " hold must be a non-negative, finite time");
        }
    }

    if (targets.empty()) {
        return {};
    }

    std::vector<VelocitySample> samples;
    const auto emit = [&](double velocity) {
        const double previous = samples.back().velocity;
        const double time = static_cast<double>(samples.size()) * config.dt;
        samples.push_back(VelocitySample{ velocity, (velocity - previous) / config.dt, time });
    };
    samples.push_back(VelocitySample{ startVelocity, 0.0, 0.0 });

    const double maxStep = maxAccel * config.dt;
    for (const VelocityTarget& target : targets) {
        // Each ramp step is placed from where the ramp began rather than added
        // to the one before, so rounding cannot accumulate and a large velocity
        // cannot absorb a small step. The final step covers only what is left,
        // landing exactly on the target.
        const double from = samples.back().velocity;
        const double gap = target.velocity - from;
        const long rampSteps = stepsToCover(std::fabs(gap), maxStep);
        for (long k = 1; k < rampSteps; ++k) {
            emit(from + std::copysign(maxStep * static_cast<double>(k), gap));
        }
        if (rampSteps > 0) {
            emit(target.velocity);
        }

        const long holdSteps = stepsToCover(target.hold, config.dt);
        for (long k = 0; k < holdSteps; ++k) {
            emit(target.velocity);
        }
    }
    return samples;
}
