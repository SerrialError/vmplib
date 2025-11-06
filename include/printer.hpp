// printer.hpp
#pragma once

#include <vector>
#include <string>
#include "types.hpp"

namespace Printer {

    // Prints: label [ (x1,y1), (x2,y2), … ]\n
    void printPoseVectorDesmos(const std::string& label, const std::vector<std::vector<Pose>>& poses);
    void printPoseVectorCode(const std::string& label, const std::vector<std::vector<Pose>>& poses);

    // Prints either “linear” or “angular” component as: label [ (t1, value1), (t2, value2), … ]\n
    void printVelocityVectorDesmos(const std::string& label,
                             const std::vector<std::vector<VelocityLayout>>& vels,
                             const std::string& whichField);
    void printVelocityVectorCode(const std::string& label,
                             const std::vector<std::vector<VelocityLayout>>& vels);
}
