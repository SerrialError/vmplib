#include "file-parser.hpp"
#include "bezier.hpp"
#include <fstream>
#include <limits>
#include <sstream>

namespace {

// Degree-elevate a straight line (its two endpoints) to the equivalent cubic,
// with control handles at the 1/3 and 2/3 marks. The curve is identical to the
// line, so downstream curvature is zero, but every segment is now a valid
// 4-point cubic and the [[segment-validation]] contract holds unchanged.
std::vector<Point> lineToCubic(const Point& a, const Point& b) {
    return {a,
            {a.x + (b.x - a.x) / 3.0, a.y + (b.y - a.y) / 3.0},
            {a.x + 2.0 * (b.x - a.x) / 3.0, a.y + 2.0 * (b.y - a.y) / 3.0},
            b};
}

// A path.jerryio points block is a Bezier spline flattened to shared-endpoint
// control points: 3N+1 of them for N joined cubics, or 2 for a straight line.
// Split it into the individual 4-point cubic segments the profiler expects.
std::vector<std::vector<Point>> splitIntoSegments(const std::vector<Point>& pts) {
    if (pts.size() == 2) {
        return {lineToCubic(pts[0], pts[1])};
    }
    if (pts.size() >= 4 && (pts.size() - 1) % 3 == 0) {
        std::vector<std::vector<Point>> segments;
        for (size_t i = 0; i + 3 < pts.size(); i += 3) {
            segments.push_back({pts[i], pts[i + 1], pts[i + 2], pts[i + 3]});
        }
        return segments;
    }
    throw FileParseError(
        "a points block must be a straight line (2 points) or a Bezier spline "
        "(3N+1 points); got " + std::to_string(pts.size()));
}

// Route each of a block's keyframes to the sub-segment whose curve it lies
// closest to. Keeps controlPoints and keyFrameVelocityList parallel per segment
// once a block splits into several, so convertToTFrame later projects each
// keyframe onto the right piece instead of failing it as off-path.
std::vector<std::vector<KeyframeVelocitiesXandY>> distributeKeyframes(
    const std::vector<std::vector<Point>>& segments,
    const std::vector<KeyframeVelocitiesXandY>& keyframes
) {
    std::vector<std::vector<KeyframeVelocitiesXandY>> perSegment(segments.size());
    for (const auto& kf : keyframes) {
        size_t best = 0;
        double bestResidual = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < segments.size(); ++i) {
            double residual = 0.0;
            projectOntoCurve(segments[i], kf.x, kf.y, &residual);
            if (residual < bestResidual) {
                bestResidual = residual;
                best = i;
            }
        }
        perSegment[best].push_back(kf);
    }
    return perSegment;
}

} // namespace

void loadPaths(
    const std::string& filename,
    std::vector<std::vector<Point>>& controlPoints,
    std::vector<std::vector<KeyframeVelocitiesXandY>>& keyFrameVelocityList
) {
    std::ifstream infile(filename);
    if (!infile) {
        throw FileParseError("could not open " + filename);
    }

    enum class State { None, ReadingPoints, ReadingVels };
    State state = State::None;

    std::string line;
    std::vector<Point>    currentPoints;
    std::vector<KeyframeVelocitiesXandY> currentVels;

    auto pushBlock = [&]() {
        if (!currentPoints.empty()) {
            // One block is a whole spline; split it into its cubic segments and
            // route the block's keyframes to the segment each one sits on. This
            // keeps the two lists aligned per segment: a segment with no
            // keyframes gets an empty list, not a placeholder at the origin that
            // would project onto an arbitrary point.
            std::vector<std::vector<Point>> segments = splitIntoSegments(currentPoints);
            std::vector<std::vector<KeyframeVelocitiesXandY>> perSegment =
                distributeKeyframes(segments, currentVels);
            for (size_t i = 0; i < segments.size(); ++i) {
                controlPoints.push_back(std::move(segments[i]));
                keyFrameVelocityList.push_back(std::move(perSegment[i]));
            }
            currentPoints.clear();
            currentVels.clear();
        }
    };

    while (std::getline(infile, line)) {
        // trim leading whitespace
        auto first = line.find_first_not_of(" \t");
        if (first == std::string::npos) continue;
        line = line.substr(first);

        if (line.rfind("#PATH-START", 0) == 0) {
            // if we were mid‑block, push it
            if (state == State::ReadingVels) {
                pushBlock();
            }
            state = State::None;
            continue;
        }
        if (line.rfind("#PATH.JERRYIO-DATA", 0) == 0) {
            // if we were mid‑block, push it
            if (state == State::ReadingVels) {
                pushBlock();
            }
            state = State::None;
            continue;
        }
        if (line.rfind("#POINTS-START", 0) == 0) {
            // if we just finished a velocities block, push it before starting a new one
            if (state == State::ReadingVels) {
                pushBlock();
            }
            currentPoints.clear();
            state = State::ReadingPoints;
            continue;
        }
        if (line.rfind("#VELOCITIES-START", 0) == 0) {
            state = State::ReadingVels;
            currentVels.clear();
            continue;
        }

        std::istringstream iss(line);
        if (state == State::ReadingPoints) {
            double x, y;
            char comma;
            if (iss >> x >> comma >> y && comma == ',') {
                currentPoints.push_back(Point{x, y});
            } else {
                throw FileParseError("failed to parse point line: " + line);
            }
        }
        else if (state == State::ReadingVels) {
            double vx, vy, vz;
            char c1, c2;
            if (iss >> vx >> c1 >> vy >> c2 >> vz && c1 == ',' && c2 == ',') {
                currentVels.push_back(KeyframeVelocitiesXandY{vx, vy, vz});
            } else {
                throw FileParseError("failed to parse velocity line: " + line);
            }
        }
    }

    // at EOF, if we were mid‑block, push it
    if (state == State::ReadingVels) {
        pushBlock();
    }
}
