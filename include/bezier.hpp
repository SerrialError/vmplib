#pragma once
#include <cmath>
#include <vector>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <limits>

struct Point {
    float x, y;
};

struct Pose {
    float x, y, theta;
};

struct Velocities {
    float linear, angular;
};

struct VelocityLayout {
    float linear, angular, time;
};

struct KeyframeVelocities {
    float velocity, time;
};

struct KeyframeVelocitiesXandY {
    float x, y, velocity;
};

class FirstOrderWithDeadBand {
public:
    FirstOrderWithDeadBand(float K, float tau, float u_static);
    float update(float u, float dt);
    void reset();

private:
    float K, tau, u_static;
    float y, dy;
};

Point bezierDerivative(const std::vector<Point>& controlPoints, float t);
Point bezierSecondDerivative(const std::vector<Point>& controlPoints, float t);
float speed(const std::vector<Point>& controlPoints, float t);
float arcLength(const std::vector<Point>& controlPoints, float a, float b);
float sFunction(const std::vector<Point>& controlPoints, float t);
float findTForS(const std::vector<Point>& controlPoints, float sCurrent, float deltaS);

float bezierComponent(const std::vector<Point>& controlPoints, float t, bool useX);
bool tryNewtonRaphson(const std::vector<Point>& controlPoints, float target, bool useX, float& t);
float findTForComponent(const std::vector<Point>& controlPoints, float target, bool useX, float prevT = 0.5f);

std::vector<KeyframeVelocities> convertToTFrame(
    const std::vector<Point>& bezierPoints,
    const std::vector<KeyframeVelocitiesXandY>& keyFrameVelocitiesXY
);

float curvature(const std::vector<Point>& controlPoints, float t);
void printVels(std::string splineName, const std::vector<Point>& controlPoints, std::vector<KeyframeVelocitiesXandY> keyFrameVelocityInitList, bool useKeyFrames);
