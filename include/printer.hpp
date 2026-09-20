// printer.hpp
#pragma once

#include <iosfwd>
#include <string>
#include <vector>
#include "drive-samples.hpp"
#include "scalar-profile.hpp"
#include "types.hpp"
#include "velocity-profiler.hpp"

namespace Printer {

    // Prints: label [ (x1,y1), (x2,y2), … ]\n
    void printPoseVectorDesmos(std::ostream& out, const std::string& label,
                               const std::vector<std::vector<Pose>>& poses);
    void printPoseVectorCpp(std::ostream& out, const std::string& label,
                             const std::vector<std::vector<Pose>>& poses);

    // Prints either “linear” or “angular” component as: label [ (t1, value1), (t2, value2), … ]\n
    void printVelocityVectorDesmos(std::ostream& out, const std::string& label,
                                   const std::vector<std::vector<VelocityLayout>>& vels,
                                   const std::string& whichField);
    void printVelocityVectorCpp(std::ostream& out, const std::string& label,
                                 const std::vector<std::vector<VelocityLayout>>& vels);

    // 1D samples as three Desmos lists against time:
    //   P = [(t, position), ...], V = [(t, velocity), ...], A = [(t, accel), ...]
    void printScalarSamplesDesmos(std::ostream& out, const std::vector<ScalarSample>& samples);

    // 1D samples as a C++ initialiser list, one {position, velocity, accel} per dt:
    //   S = {{p, v, a},{p, v, a},...};
    void printScalarSamplesCpp(std::ostream& out, const std::vector<ScalarSample>& samples);

    // Velocity samples as two Desmos lists against time:
    //   V = [(t, velocity), ...], A = [(t, accel), ...]
    void printVelocitySamplesDesmos(std::ostream& out, const std::vector<VelocitySample>& samples);

    // Velocity samples as a C++ initialiser list, one {velocity, accel} per dt:
    //   S = {{v, a},{v, a},...};
    void printVelocitySamplesCpp(std::ostream& out, const std::vector<VelocitySample>& samples);

    // The Rust format: a module holding a
    //   pub static SAMPLES: &[...] = &[...];
    // slice of one sample per dt, every value at full f64 precision. Each
    // sample's acceleration is the one that carries it to the next sample, so a
    // controller holding a sample for one dt feeds it forward as is; the last
    // sample carries 0.
    //
    // typePath is the Rust path of a sample type the crate this file is dropped
    // into already defines, such as "crate::motion_profile::DriveSample". The
    // file then writes its samples as that type and imports it, which is what
    // lets one follower take every profile in a crate: a struct declared per
    // file would be a separate type per file. Empty declares the struct in the
    // file instead, which suits a crate holding a single profile. A path with no
    // "::" names a type already in scope and is imported.

    // 1D samples as MotionSample { time, velocity, accel }. Position is left
    // out.
    void printScalarSamplesRust(std::ostream& out, const std::vector<ScalarSample>& samples,
                                const std::string& typePath = {});

    // Velocity samples as the same MotionSample { time, velocity, accel }.
    void printVelocitySamplesRust(std::ostream& out, const std::vector<VelocitySample>& samples,
                                  const std::string& typePath = {});

    // Drive samples as DriveSample { time, linear_velocity, angular_velocity,
    // left_velocity, right_velocity, left_accel, right_accel }.
    void printDriveSamplesRust(std::ostream& out, const std::vector<DriveSample>& samples,
                               const std::string& typePath = {});
}
