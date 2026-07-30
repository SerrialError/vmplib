#include "motion-profiler.hpp"
#include "file-parser.hpp"
#include "printer.hpp"
#include "types.hpp"
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace {

void printUsage() {
    std::cerr << "Usage: ./main --file <path> [--out <path>] [--format desmos|code]\n";
}

void writeTrajectory(std::ostream& out, const Trajectory& traj, const std::string& format) {
    if (format == "desmos") {
        Printer::printPoseVectorDesmos(out, "X = ", traj.poses);
        Printer::printVelocityVectorDesmos(out, "L = ", traj.velocities, "linear");
        Printer::printVelocityVectorDesmos(out, "A = ", traj.velocities, "angular");
        Printer::printPoseVectorDesmos(out, "X_r = ", traj.followedPoses);
        Printer::printVelocityVectorDesmos(out, "L_r = ", traj.followedVelocities, "linear");
        Printer::printVelocityVectorDesmos(out, "A_r = ", traj.followedVelocities, "angular");
    } else {
        Printer::printPoseVectorCode(out, "P =", traj.poses);
        Printer::printVelocityVectorCode(out, "V =", traj.velocities);
    }
}

} // namespace

int main(int argc, char* argv[]) {
    std::string filename;
    std::string outPath = "output.txt";
    std::string format = "desmos";

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--file" && i + 1 < argc) {
            filename = argv[++i];
        } else if (arg == "--out" && i + 1 < argc) {
            outPath = argv[++i];
        } else if (arg == "--format" && i + 1 < argc) {
            format = argv[++i];
        } else {
            std::cerr << "error: unrecognised argument '" << arg << "'\n";
            printUsage();
            return 1;
        }
    }

    if (filename.empty()) {
        printUsage();
        return 1;
    }
    if (format != "desmos" && format != "code") {
        std::cerr << "error: --format must be 'desmos' or 'code'\n";
        return 1;
    }

    try {
        std::vector<std::vector<Point>> controlPoints;
        std::vector<std::vector<KeyframeVelocitiesXandY>> keyframeList;
        loadPaths(filename, controlPoints, keyframeList);

        const Trajectory traj = generateTrajectory(controlPoints, keyframeList, true);

        std::ofstream out(outPath);
        if (!out) {
            std::cerr << "error: cannot open " << outPath << " for writing\n";
            return 1;
        }
        writeTrajectory(out, traj, format);
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
