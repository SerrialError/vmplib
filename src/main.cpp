#include "motion-profiler.hpp"
#include "file-parser.hpp"
#include "printer.hpp"
#include "types.hpp"
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace {

void printUsage() {
    std::cerr << "Usage: ./main --file <path> --max-vel <m/s> --max-accel <m/s^2>\n"
                 "              --track-width <m> [--dt <s>] [--out <path>]\n"
                 "              [--format desmos|code]\n";
}

// Reads the whole of text as a number, so "1.5m" or "" is an error instead of a
// silently truncated 1.5 or 0.
std::optional<double> parseNumber(const std::string& text) {
    try {
        size_t used = 0;
        const double value = std::stod(text, &used);
        if (used == text.size()) {
            return value;
        }
    } catch (const std::exception&) {
    }
    return std::nullopt;
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
    ProfileConfig config;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const bool hasValue = i + 1 < argc;
        if (arg == "--file" && hasValue) {
            filename = argv[++i];
        } else if (arg == "--out" && hasValue) {
            outPath = argv[++i];
        } else if (arg == "--format" && hasValue) {
            format = argv[++i];
        } else if ((arg == "--max-vel" || arg == "--max-accel" || arg == "--track-width" ||
                    arg == "--dt") && hasValue) {
            const std::string text = argv[++i];
            const std::optional<double> value = parseNumber(text);
            if (!value) {
                std::cerr << "error: " << arg << " expects a number, got '" << text << "'\n";
                return 1;
            }
            if (arg == "--max-vel") {
                config.maxVelocity = *value;
            } else if (arg == "--max-accel") {
                config.maxAccel = *value;
            } else if (arg == "--track-width") {
                config.trackWidth = *value;
            } else {
                config.dt = *value;
            }
        } else {
            std::cerr << "error: unrecognised argument '" << arg << "'\n";
            printUsage();
            return 1;
        }
    }

    // Report every missing flag at once rather than making the user rerun once
    // per flag.
    std::vector<std::string> missing;
    if (filename.empty()) missing.push_back("--file");
    if (!config.maxVelocity) missing.push_back("--max-vel");
    if (!config.maxAccel) missing.push_back("--max-accel");
    if (!config.trackWidth) missing.push_back("--track-width");
    if (!missing.empty()) {
        std::cerr << "error: missing required";
        for (const std::string& flag : missing) {
            std::cerr << " " << flag;
        }
        std::cerr << "\n";
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

        const Trajectory traj = generateTrajectory(controlPoints, keyframeList, true, config);

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
