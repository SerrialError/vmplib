#include "motion-profiler.hpp"
#include "file-parser.hpp"
#include "types.hpp"
#include <vector>
#include <iostream>
#include <string>

int main(int argc, char* argv[]) {    
	std::vector<std::vector<Point>> controlPoints;
	std::vector<std::vector<KeyframeVelocitiesXandY>> keyFrameVelocityList;
	std::string filename;

        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];

            if (arg == "--file" && i + 1 < argc) {
                filename = argv[i + 1];
                ++i; // Skip next because it's the filename
            }
        }

        if (!filename.empty()) {
            std::cout << "Using file: " << filename << std::endl;
        } else {
            std::cerr << "Usage: ./main --file <filename>\n";
            return 1;
        }
        loadPaths(filename, controlPoints, keyFrameVelocityList);
	printVels(controlPoints, keyFrameVelocityList, true);
	return(0);
}
