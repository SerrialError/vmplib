#pragma once

#include <vector>
#include "motion-profiler.hpp"

// One planned sample of a differential drive, in the terms its velocity
// controller works in: the robot's speed and turn rate, and what each side of
// the drivetrain has to do to produce them.
struct DriveSample {
    double time;            // s
    double linear;          // m/s, at the centre of the robot
    double angular;         // rad/s, counterclockwise positive
    double leftVelocity;    // m/s
    double rightVelocity;   // m/s
    // The acceleration that carries each side from this sample to the next,
    // (next velocity - this velocity) / dt, and 0 at the last sample. A
    // controller that holds this sample for one dt feeds it forward as is. This
    // is the opposite pairing to ScalarSample::accel, which is the acceleration
    // that arrived at its sample.
    double leftAccel;       // m/s^2
    double rightAccel;      // m/s^2
};

// The planned trajectory, every segment in order, as one drive sample per dt.
// Each side runs at v -+ omega * trackWidth / 2, with trackWidth and dt taken
// from the config the trajectory was generated with.
//
// Throws ConfigError (config-error.hpp) if config's trackWidth is unset, or it
// or dt is not positive and finite.
std::vector<DriveSample> driveSamples(const Trajectory& traj, const ProfileConfig& config);
