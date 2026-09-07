// motion-utils.hpp
#pragma once

#include <vector>
#include <cmath>
#include "types.hpp"

// Geometry helpers that are not tied to a Bezier live here; the curve
// routines themselves are declared in bezier.hpp.
namespace MotionUtils {

    // Wraps an angle into [-pi, +pi]
    inline double wrapAngle(double angle) {
        double wrapped = std::fmod(angle + M_PI, 2.0 * M_PI);
        if (wrapped < 0) wrapped += 2.0 * M_PI;
        return wrapped - M_PI;
    }

    // Sinc function: sin(x)/x, with limit →1 as x→0
    inline double sinc(double x) {
        return (std::abs(x) < 1e-9) ? 1.0 : std::sin(x) / x;
    }
}
