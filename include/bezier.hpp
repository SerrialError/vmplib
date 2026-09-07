#pragma once
#include "types.hpp"
#include <vector>
#include <stdexcept>
#include <string>

// Thrown for a path segment whose control-point count is not one this library
// can evaluate. Every bezier* routine indexes controlPoints[0..3]
// unconditionally, so a segment with the wrong count is a memory-safety bug
// rather than a numerical one; it is caught here at the boundary before any of
// them run.
class SegmentError : public std::runtime_error {
public:
    explicit SegmentError(const std::string& what) : std::runtime_error(what) {}
};

// Throws SegmentError unless controlPoints holds exactly the four points a cubic
// Bezier segment requires. Call this before handing a segment to any bezier*
// routine or to a profile.
void requireCubicSegment(const std::vector<Point>& controlPoints);

Point bezierDerivative(const std::vector<Point>& controlPoints, double t);
Point bezierSecondDerivative(const std::vector<Point>& controlPoints, double t);
double speed(const std::vector<Point>& controlPoints, double t);
double arcLength(const std::vector<Point>& controlPoints, double a, double b);
double sFunction(const std::vector<Point>& controlPoints, double t);
// tGuess seeds the Newton iteration; pass the previous parameter when stepping
// along a path so the solver starts within one timestep of the answer.
double findTForS(const std::vector<Point>& controlPoints, double sCurrent, double deltaS,
                 double tGuess = 0.5);

Pose findXandY(const std::vector<Point>& controlPoints, double t);

// Returns the t minimising ||r(t) - (x, y)||. If residual is non-null it
// receives the distance from the curve to (x, y), which is how a caller tells
// an on-path point from one that merely projects somewhere.
double projectOntoCurve(const std::vector<Point>& controlPoints, double x, double y,
                        double* residual = nullptr);

// Thrown for a keyframe that cannot be placed on the path: either too far from
// the curve to be meaningful, or out of order along it.
class KeyframeError : public std::runtime_error {
public:
    explicit KeyframeError(const std::string& what) : std::runtime_error(what) {}
};

// Throws KeyframeError rather than quietly placing a bad keyframe somewhere.
std::vector<KeyframeVelocities> convertToTFrame(
    const std::vector<Point>& bezierPoints,
    const std::vector<KeyframeVelocitiesXandY>& keyFrameVelocitiesXY
);

double signedCurvature(const std::vector<Point>& controlPoints, double t);
double unsignedCurvature(const std::vector<Point>& controlPoints, double t);
