#include "cli-args.hpp"
#include "drive-samples.hpp"
#include "motion-profiler.hpp"
#include "file-parser.hpp"
#include "mechanism-file.hpp"
#include "printer.hpp"
#include "types.hpp"
#include "velocity-profiler.hpp"
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
    "              [--dt <s>] [--out <path>] [--format desmos|cpp|rust]\n"
    "  ./main --file <path> ...   same as `path`\n"
    "  ./main linear --file <moves> --max-vel <units/s> --max-accel <units/s^2>\n"
    "              [--dt <s>] [--out <path>] [--format desmos|cpp|rust]\n"
    "  ./main velocity --file <targets> --max-accel <units/s^2> [--max-vel <units/s>]\n"
    "              [--dt <s>] [--out <path>] [--format desmos|cpp|rust]\n"
    "  ./main --help\n"
    "\n"
    "  --rust-type <path>   with --format rust, write the samples as a type the\n"
    "                       crate already defines, such as\n"
    "                       crate::motion_profile::DriveSample, and import it\n"
    "                       rather than declaring a struct in the file\n";

// --format, checked against the styles every mode can print.
std::string outputFormat(const FlagMap& flags) {
    const std::string format = stringFlag(flags, "--format", "desmos");
    if (format != "desmos" && format != "cpp" && format != "rust") {
        throw CliError("--format must be 'desmos', 'cpp' or 'rust'");
    }
    return format;
}

// --rust-type: the sample type the Rust output writes its samples as, empty
// unless given. One type shared across a crate's profiles is what lets a single
// follower take all of them, since a struct declared per file is a type per
// file.
std::string rustSampleType(const FlagMap& flags, const std::string& format) {
    const std::string path = rustPathFlag(flags, "--rust-type");
    if (!path.empty() && format != "rust") {
        throw CliError("--rust-type only applies to --format rust");
    }
    return path;
}

std::ofstream openOutput(const FlagMap& flags) {
    const std::string path = stringFlag(flags, "--out", "output.txt");
    std::ofstream out(path);
    if (!out) {
        throw std::runtime_error("cannot open " + path + " for writing");
    }
    return out;
}

void writeTrajectory(std::ostream& out, const Trajectory& traj, const ProfileConfig& config,
                     const std::string& format, const std::string& rustType) {
    if (format == "desmos") {
        Printer::printPoseVectorDesmos(out, "X = ", traj.poses);
        Printer::printVelocityVectorDesmos(out, "L = ", traj.velocities, "linear");
        Printer::printVelocityVectorDesmos(out, "A = ", traj.velocities, "angular");
        Printer::printPoseVectorDesmos(out, "X_r = ", traj.followedPoses);
        Printer::printVelocityVectorDesmos(out, "L_r = ", traj.followedVelocities, "linear");
        Printer::printVelocityVectorDesmos(out, "A_r = ", traj.followedVelocities, "angular");
    } else if (format == "cpp") {
        Printer::printPoseVectorCpp(out, "P =", traj.poses);
        Printer::printVelocityVectorCpp(out, "V =", traj.velocities);
    } else {
        Printer::printDriveSamplesRust(out, driveSamples(traj, config), rustType);
    }
}

// A differential drive along the Bezier path in --file.
void runPath(const std::vector<std::string>& args) {
    const FlagMap flags = parseFlags(args, {"--file", "--max-vel", "--max-accel",
                                            "--track-width", "--dt", "--out", "--format",
                                            "--rust-type"});
    requireFlags(flags, {"--file", "--max-vel", "--max-accel", "--track-width"});
    const std::string format = outputFormat(flags);
    const std::string rustType = rustSampleType(flags, format);

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
    writeTrajectory(out, traj, config, format, rustType);
}

// A single axis -- lift, arm, turret, straight drive -- through the moves in
// --file.
void runLinear(const std::vector<std::string>& args) {
    const FlagMap flags = parseFlags(args, {"--file", "--max-vel", "--max-accel", "--dt",
                                            "--out", "--format", "--rust-type"});
    requireFlags(flags, {"--file", "--max-vel", "--max-accel"});
    const std::string format = outputFormat(flags);
    const std::string rustType = rustSampleType(flags, format);

    ScalarProfileConfig config;
    config.maxVelocity = numberFlag(flags, "--max-vel");
    config.maxAccel = numberFlag(flags, "--max-accel");
    if (const auto dt = numberFlag(flags, "--dt")) {
        config.dt = *dt;
    }

    const MoveFile file = loadMoves(flags.at("--file"));
    const std::vector<ScalarSample> samples =
        generateScalarMoves(config, file.startPosition, file.moves);

    std::ofstream out = openOutput(flags);
    if (format == "desmos") {
        Printer::printScalarSamplesDesmos(out, samples);
    } else if (format == "cpp") {
        Printer::printScalarSamplesCpp(out, samples);
    } else {
        Printer::printScalarSamplesRust(out, samples, rustType);
    }
}

// A mechanism commanded by speed -- flywheel, roller, intake -- through the
// velocity targets in --file.
void runVelocity(const std::vector<std::string>& args) {
    const FlagMap flags = parseFlags(args, {"--file", "--max-accel", "--max-vel", "--dt",
                                            "--out", "--format", "--rust-type"});
    requireFlags(flags, {"--file", "--max-accel"});
    const std::string format = outputFormat(flags);
    const std::string rustType = rustSampleType(flags, format);

    VelocityProfileConfig config;
    config.maxAccel = numberFlag(flags, "--max-accel");
    config.maxVelocity = numberFlag(flags, "--max-vel");
    if (const auto dt = numberFlag(flags, "--dt")) {
        config.dt = *dt;
    }

    const std::vector<VelocitySample> samples =
        generateVelocityProfile(config, loadVelocityTargets(flags.at("--file")));

    std::ofstream out = openOutput(flags);
    if (format == "desmos") {
        Printer::printVelocitySamplesDesmos(out, samples);
    } else if (format == "cpp") {
        Printer::printVelocitySamplesCpp(out, samples);
    } else {
        Printer::printVelocitySamplesRust(out, samples, rustType);
    }
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
        } else if (mode == "linear") {
            runLinear({args.begin() + 1, args.end()});
        } else if (mode == "velocity") {
            runVelocity({args.begin() + 1, args.end()});
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
