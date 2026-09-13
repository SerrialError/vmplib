// printer.hpp
#pragma once

#include <iosfwd>
#include <string>
#include <vector>
#include "scalar-profile.hpp"
#include "types.hpp"
#include "velocity-profiler.hpp"

namespace Printer {

    // Prints: label [ (x1,y1), (x2,y2), … ]\n
    void printPoseVectorDesmos(std::ostream& out, const std::string& label,
                               const std::vector<std::vector<Pose>>& poses);
    void printPoseVectorCode(std::ostream& out, const std::string& label,
                             const std::vector<std::vector<Pose>>& poses);

    // Prints either “linear” or “angular” component as: label [ (t1, value1), (t2, value2), … ]\n
    void printVelocityVectorDesmos(std::ostream& out, const std::string& label,
                                   const std::vector<std::vector<VelocityLayout>>& vels,
                                   const std::string& whichField);
    void printVelocityVectorCode(std::ostream& out, const std::string& label,
                                 const std::vector<std::vector<VelocityLayout>>& vels);

    // 1D samples as three Desmos lists against time:
    //   P = [(t, position), ...], V = [(t, velocity), ...], A = [(t, accel), ...]
    void printScalarSamplesDesmos(std::ostream& out, const std::vector<ScalarSample>& samples);

    // 1D samples as a C++ initialiser list, one {position, velocity, accel} per dt:
    //   S = {{p, v, a},{p, v, a},...};
    void printScalarSamplesCode(std::ostream& out, const std::vector<ScalarSample>& samples);

    // Velocity samples as two Desmos lists against time:
    //   V = [(t, velocity), ...], A = [(t, accel), ...]
    void printVelocitySamplesDesmos(std::ostream& out, const std::vector<VelocitySample>& samples);

    // Velocity samples as a C++ initialiser list, one {velocity, accel} per dt:
    //   S = {{v, a},{v, a},...};
    void printVelocitySamplesCode(std::ostream& out, const std::vector<VelocitySample>& samples);
}
