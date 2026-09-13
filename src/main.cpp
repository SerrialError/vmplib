#include "cli-args.hpp"
#include "motion-profiler.hpp"
#include "file-parser.hpp"
#include "printer.hpp"
#include "types.hpp"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr const char* kUsage =
    "Usage:\n"
    "  ./main path --file <path> --max-vel <m/s> --max-accel <m/s^2> --track-width <m>\n"
    "              [--dt <s>] [--out <path>] [--format desmos|code]\n"
    "  ./main --file <path> ...   same as `path`\n"
    "  ./main --help\n";

// --format, checked against the styles every mode can print.
std::string outputFormat(const FlagMap& flags) {
    const std::string format = stringFlag(flags, "--format", "desmos");
    if (format != "desmos" && format != "code") {
        throw CliError("--format must be 'desmos' or 'code'");
    }
    return format;
}

std::ofstream openOutput(const FlagMap& flags) {
    const std::string path = stringFlag(flags, "--out", "output.txt");
    std::ofstream out(path);
    if (!out) {
        throw std::runtime_error("cannot open " + path + " for writing");
    }
    return out;
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

// A differential drive along the Bezier path in --file.
void runPath(const std::vector<std::string>& args) {
    const FlagMap flags = parseFlags(args, {"--file", "--max-vel", "--max-accel",
                                            "--track-width", "--dt", "--out", "--format"});
    requireFlags(flags, {"--file", "--max-vel", "--max-accel", "--track-width"});
    const std::string format = outputFormat(flags);

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

    std::ofstream out = openOutput(flags);
    writeTrajectory(out, traj, format);
}

} // namespace

int main(int argc, char* argv[]) {
    const std::vector<std::string> args(argv + 1, argv + argc);

    // A bare "help" only counts as the mode; anywhere else it could be a value,
    // such as --out help.
    const auto isHelpFlag = [](const std::string& arg) { return arg == "--help" || arg == "-h"; };
    if (std::any_of(args.begin(), args.end(), isHelpFlag) ||
        (!args.empty() && args.front() == "help")) {
        std::cout << kUsage;
        return 0;
    }

    try {
        if (args.empty()) {
            throw CliError("no mode given");
        }
        const std::string& mode = args.front();
        if (mode == "path") {
            runPath({args.begin() + 1, args.end()});
        } else if (mode.rfind("--", 0) == 0) {
            // The original form, `./main --file ...`, predates modes and still
            // means `path`.
            runPath(args);
        } else {
            throw CliError("unknown mode '" + mode + "'");
        }
    } catch (const CliError& e) {
        std::cerr << "error: " << e.what() << "\n" << kUsage;
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
