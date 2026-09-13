#include "cli-args.hpp"
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
    std::cerr << "Usage: ./main --file <path> --max-vel <m/s> --max-accel <m/s^2>\n"
                 "              --track-width <m> [--dt <s>] [--out <path>]\n"
                 "              [--format desmos|code]\n";
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
    const std::vector<std::string> args(argv + 1, argv + argc);

    try {
        const FlagMap flags = parseFlags(args, {"--file", "--max-vel", "--max-accel",
                                                "--track-width", "--dt", "--out", "--format"});
        requireFlags(flags, {"--file", "--max-vel", "--max-accel", "--track-width"});

        const std::string format = stringFlag(flags, "--format", "desmos");
        if (format != "desmos" && format != "code") {
            throw CliError("--format must be 'desmos' or 'code'");
        }

        ProfileConfig config;
        config.maxVelocity = numberFlag(flags, "--max-vel");
        config.maxAccel = numberFlag(flags, "--max-accel");
        config.trackWidth = numberFlag(flags, "--track-width");
        if (const auto dt = numberFlag(flags, "--dt")) {
            config.dt = *dt;
        }

        std::vector<std::vector<Point>> controlPoints;
        std::vector<std::vector<KeyframeVelocitiesXandY>> keyframeList;
        loadPaths(flags.at("--file"), controlPoints, keyframeList);

        const Trajectory traj = generateTrajectory(controlPoints, keyframeList, true, config);

        const std::string outPath = stringFlag(flags, "--out", "output.txt");
        std::ofstream out(outPath);
        if (!out) {
            std::cerr << "error: cannot open " << outPath << " for writing\n";
            return 1;
        }
        writeTrajectory(out, traj, format);
    } catch (const CliError& e) {
        std::cerr << "error: " << e.what() << "\n";
        printUsage();
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
