// printer.cpp
#include "printer.hpp"
#include <iostream>
#include <iomanip>

namespace Printer {

    void printPoseVector(const std::string& label, const std::vector<Pose>& poses) {
        std::cout << label << "[";
        for (size_t i = 0; i < poses.size(); ++i) {
            std::cout << "("
                      << std::fixed << std::setprecision(6)
                      << poses[i].x << "," << poses[i].y << ")";
            if (i + 1 < poses.size()) std::cout << ",";
        }
        std::cout << "]\n";
    }

    void printVelocityVector(const std::string& label,
                             const std::vector<VelocityLayout>& vels,
                             const std::string& whichField)
    {
        std::cout << label << "[";
        for (size_t i = 0; i < vels.size(); ++i) {
            float value = (whichField == "linear") ? vels[i].linear : vels[i].angular;
            std::cout << "("
                      << std::fixed << std::setprecision(6)
                      << vels[i].time << "," << value << ")";
            if (i + 1 < vels.size()) std::cout << ",";
        }
        std::cout << "]\n";
    }
}
