// Minimal 1D profiling example.
//
// Profiles a straight 2 m move that slows to 0.4 m/s halfway through, then
// stops at the end, and prints the resulting samples. The library is
// unit-agnostic; here everything is metres and seconds.
//
// Build and run:
//   make example
//   ./bin/scalar-profile-demo

#include "scalar-profiler.hpp"

#include <cstdio>
#include <vector>

int main() {
    constexpr double maxVelocity = 1.5;  // m/s
    const ScalarProfileConfig config{
        maxVelocity,
        /*maxAccel=*/3.0,      // m/s^2
        /*dt=*/0.01,           // s
    };

    // Slow to 0.4 m/s at the 1 m mark, then release back to full speed.
    const std::vector<ScalarKeyframe> keyframes = {
        {maxVelocity, 0.0},
        {0.4, 1.0},
        {maxVelocity, 2.0},
    };

    const std::vector<ScalarSample> samples =
        generateScalarProfile(/*distance=*/2.0, config, /*startVel=*/0.0,
                              /*endVel=*/0.0, keyframes);

    std::printf("%6s  %10s  %10s  %10s  %10s\n", "i", "time(s)", "pos(m)", "vel(m/s)",
                "acc(m/s^2)");
    // Print every 20th sample so the output stays short, plus the last one.
    for (size_t i = 0; i < samples.size(); ++i) {
        if (i % 20 != 0 && i + 1 != samples.size()) {
            continue;
        }
        const ScalarSample& s = samples[i];
        std::printf("%6zu  %10.3f  %10.4f  %10.4f  %10.4f\n", i, s.time, s.position,
                    s.velocity, s.accel);
    }
    std::printf("\n%zu samples, ends at %.4f m/s\n", samples.size(),
                samples.empty() ? 0.0 : samples.back().velocity);
    return 0;
}
