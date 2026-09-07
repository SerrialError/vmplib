#pragma once
// Internal state is carried in double throughout the library: arc length,
// accumulated time and pose, and the Newton/quadrature intermediates all
// accumulate over hundreds of steps, where float rounding is visible. Narrowing
// to float happens only at the output boundary (see printer.cpp).
struct Point {
    double x, y;
};

struct Pose {
    double x, y, theta;
};

struct Velocities {
    double linear, angular;
};

struct VelocityLayout {
    double linear, angular, time;
};

// A target velocity pinned to a Bezier parameter t, not to a wall-clock time.
struct KeyframeVelocities {
    double velocity, t;
};

struct KeyframeVelocitiesXandY {
    double x, y, velocity;
};
