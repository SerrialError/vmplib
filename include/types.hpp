#pragma once
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

// A target velocity pinned to a Bezier parameter t, not to a wall-clock time.
struct KeyframeVelocities {
    float velocity, t;
};

struct KeyframeVelocitiesXandY {
    float x, y, velocity;
};
