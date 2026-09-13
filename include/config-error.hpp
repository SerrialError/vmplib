#pragma once

#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

// Thrown when a motion limit is not a positive, finite number. Nothing
// downstream fails loudly on a bad limit by itself: a zero top speed never
// advances, so the profile runs until its watchdog and returns a minute of
// samples parked at the start.
class ConfigError : public std::runtime_error {
public:
    explicit ConfigError(const std::string& what) : std::runtime_error(what) {}
};

// Throws ConfigError unless value is positive and finite.
inline void requirePositiveLimit(const std::string& name, double value) {
    if (!(value > 0.0) || !std::isfinite(value)) {
        std::ostringstream msg;
        msg << name << " must be a positive, finite number; got " << value;
        throw ConfigError(msg.str());
    }
}
