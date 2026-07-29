// motion-utils.hpp
#pragma once

#include <vector>
#include <cmath>
#include "types.hpp"

// Geometry helpers that are not tied to a Bezier live here; the curve
// routines themselves are declared in bezier.hpp.
namespace MotionUtils {

    // Wraps an angle into [-pi, +pi]
    inline float wrapAngle(float angle) {
        float wrapped = std::fmod(angle + static_cast<float>(M_PI), 2.0f * static_cast<float>(M_PI));
        if (wrapped < 0) wrapped += 2.0f * static_cast<float>(M_PI);
        return wrapped - static_cast<float>(M_PI);
    }

    // Sinc function: sin(x)/x, with limit →1 as x→0
    inline float sinc(float x) {
        return (std::abs(x) < 1e-5f) ? 1.0f : std::sin(x) / x;
    }
}
