#pragma once
#include <string>
#include <vector>
#include "types.hpp"
void printVels(std::string splineName, const std::vector<Point>& controlPoints, std::vector<KeyframeVelocitiesXandY> keyFrameVelocityInitList, bool useKeyFrames);
