#include "drive-samples.hpp"
#include "config-error.hpp"   // requireLimit

std::vector<DriveSample> driveSamples(const Trajectory& traj, const ProfileConfig& config) {
    const double halfTrack = requireLimit("trackWidth", config.trackWidth) / 2.0;
    requirePositiveLimit("dt", config.dt);

    std::vector<DriveSample> samples;
    for (const auto& segment : traj.velocities) {
        for (const VelocityLayout& v : segment) {
            samples.push_back(DriveSample{ v.time, v.linear, v.angular,
                                           v.linear - v.angular * halfTrack,
                                           v.linear + v.angular * halfTrack, 0.0, 0.0 });
        }
    }

    // Only a trajectory's first segment emits its start sample, so the joined
    // list holds each join once and differencing straight across it is right.
    for (size_t i = 0; i + 1 < samples.size(); ++i) {
        samples[i].leftAccel =
            (samples[i + 1].leftVelocity - samples[i].leftVelocity) / config.dt;
        samples[i].rightAccel =
            (samples[i + 1].rightVelocity - samples[i].rightVelocity) / config.dt;
    }
    return samples;
}
