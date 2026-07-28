#pragma once
#include "types.hpp"
#include <cmath>
#include <vector>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <limits>

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

std::vector<KeyframeVelocities> convertToTFrame(
    const std::vector<Point>& bezierPoints,
    const std::vector<KeyframeVelocitiesXandY>& keyFrameVelocitiesXY
);

float signedCurvature(const std::vector<Point>& controlPoints, float t);
float unsignedCurvature(const std::vector<Point>& controlPoints, float t);
