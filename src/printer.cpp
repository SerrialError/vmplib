// printer.cpp
#include "printer.hpp"
#include <iostream>
#include <iomanip>

namespace Printer {

    void printPoseVector(
        const std::string& label,
        const std::vector<std::vector<Pose>>& poses
    ) {
        std::cout << label << "[";
        for (size_t i = 0; i < poses.size(); ++i) {
            for (size_t j = 0; j < poses[i].size(); ++j) {
                std::cout 
                    << "("
                    << std::fixed << std::setprecision(6)
                    << poses[i][j].x << "," 
                    << poses[i][j].y 
                    << ")";
                // print comma if this isn't the very last element
                if (i != poses.size() - 1 || j != poses[i].size() - 1) {
                    std::cout << ",";
                }
            }
        }
        std::cout << "]\n";
    }

    void printVelocityVector(
        const std::string& label,
        const std::vector<std::vector<VelocityLayout>>& vels,
        const std::string& whichField
    ) {
        std::cout << label << "[";
        for (size_t i = 0; i < vels.size(); ++i) {
            for (size_t j = 0; j < vels[i].size(); ++j) {
                float value = (whichField == "linear") 
                                ? vels[i][j].linear 
                                : vels[i][j].angular;
                std::cout 
                    << "("
                    << std::fixed << std::setprecision(6)
                    << vels[i][j].time << "," 
                    << value 
                    << ")";
                // same “not last element” test
                if (i != vels.size() - 1 || j != vels[i].size() - 1) {
                    std::cout << ",";
                }
            }
        }
        std::cout << "]\n";
    }
}
