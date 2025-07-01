#include "file-parser.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
void loadPaths(
    const std::string& filename,
    std::vector<std::vector<Point>>& controlPoints,
    std::vector<std::vector<KeyframeVelocitiesXandY>>& keyFrameVelocityList
) {
    std::ifstream infile(filename);
    if (!infile) {
        std::cerr << "Error: could not open " << filename << "\n";
        return;
    }

    const std::string marker = "#SPLINE-POINTS-START";
    std::string line;

    std::vector<Point> currentPoints;

    while (std::getline(infile, line)) {
        // trim leading whitespace
        auto first = line.find_first_not_of(" \t");
        if (first == std::string::npos) continue;
        line = line.substr(first);

        // start new block
        if (line.rfind(marker, 0) == 0) {
            if (!currentPoints.empty()) {
                controlPoints.push_back(currentPoints);
                // one default keyframe per path
                keyFrameVelocityList.emplace_back(1, KeyframeVelocitiesXandY{0.f, 0.f, 0.f});
                currentPoints.clear();
            }
            continue;
        }

        // parse "x, y, z"
        std::istringstream iss(line);
        float x, y, z;
        char c1, c2;
        if (iss >> x >> c1 >> y >> c2 >> z && c1 == ',' && c2 == ',') {
            currentPoints.push_back(Point{x, y});
        }
    }

    // push last block if any
    if (!currentPoints.empty()) {
        controlPoints.push_back(currentPoints);
        keyFrameVelocityList.emplace_back(1, KeyframeVelocitiesXandY{0.f, 0.f, 0.f});
    }
}
