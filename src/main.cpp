#include "motion-profiler.hpp"
#include "types.hpp"
#include <vector>
#include <string>

int main() {
	std::vector<Point> controlPoints;
	std::string splineName;
	std::vector<KeyframeVelocitiesXandY> keyFrameVelocityList;
	splineName = "spline1";
	controlPoints = {
		{ -1.585, 1.311 }, 
		{ -0.372, 1.166 }, 
		{ -1.773, -1.01 }, 
		{ -0.42, -1.212 }};
	keyFrameVelocityList = {{0,0,0}};
	printVels(splineName, controlPoints, keyFrameVelocityList, false);

	return(0);
}
