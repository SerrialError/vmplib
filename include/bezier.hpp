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

Point bezierDerivative(const std::vector<Point>& controlPoints, float t);
Point bezierSecondDerivative(const std::vector<Point>& controlPoints, float t);
float speed(const std::vector<Point>& controlPoints, float t);
float arcLength(const std::vector<Point>& controlPoints, float a, float b);
float sFunction(const std::vector<Point>& controlPoints, float t);
// tGuess seeds the Newton iteration; pass the previous parameter when stepping
// along a path so the solver starts within one timestep of the answer.
float findTForS(const std::vector<Point>& controlPoints, float sCurrent, float deltaS,
                float tGuess = 0.5f);

Pose findXandY(const std::vector<Point>& controlPoints, float t);

// Returns the t minimising ||r(t) - (x, y)||. If residual is non-null it
// receives the distance from the curve to (x, y), which is how a caller tells
// an on-path point from one that merely projects somewhere.
float projectOntoCurve(const std::vector<Point>& controlPoints, float x, float y,
                       float* residual = nullptr);

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

float signedCurvature(const std::vector<Point>& controlPoints, float t);
float unsignedCurvature(const std::vector<Point>& controlPoints, float t);
