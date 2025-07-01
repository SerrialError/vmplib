#include "motion-profiler.hpp"
#include "file-parser.hpp"
#include "types.hpp"
#include <vector>
#include <iostream>
#include <iomanip>
#include <string>

int main() {
	std::vector<std::vector<Point>> controlPoints;
	std::vector<std::vector<KeyframeVelocitiesXandY>> keyFrameVelocityList;
        loadPaths("path-points.txt", controlPoints, keyFrameVelocityList);
	printVels(controlPoints, keyFrameVelocityList, false);
	return(0);
}
