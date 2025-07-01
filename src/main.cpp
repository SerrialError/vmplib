#include "motion-profiler.hpp"
#include "types.hpp"
#include <vector>
#include <string>

int main() {
	std::vector<std::vector<Point>> controlPoints;
	std::vector<std::vector<KeyframeVelocitiesXandY>> keyFrameVelocityList;
	controlPoints = {
		{{ -1.585, 1.311 }, 
		{ -0.372, 1.166 }, 
		{ -1.773, -1.01 }, 
		{ -0.42, -1.212 }},
		{{ -1.585, 1.311 }, 
		{ -0.372, 1.166 }, 
		{ -1.773, -1.01 }, 
		{ -0.42, -1.212 }}
	};
	keyFrameVelocityList = {
		{{0,0,0}},
		{{0,0,0}}};
	printVels(controlPoints, keyFrameVelocityList, false);

	return(0);
}
