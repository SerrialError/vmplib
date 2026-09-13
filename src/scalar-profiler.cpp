#include "scalar-profiler.hpp"
#include "config-error.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace {
// The velocity ceiling is sampled on a grid this many nodes fine at most, so a
// very long move does not allocate without bound.
constexpr int kMaxCeilingNodes = 8192;

// Nodes in a velocity ceiling over length: one roughly every top-speed timestep
// of travel, so the ceiling is sampled at least as finely as the output the
// profiler emits.
int ceilingNodeCount(double length, double maxVelocity, double dt) {
    const double spacing = std::max(maxVelocity * dt, 1e-6);
    const int nodes = static_cast<int>(std::ceil(length / spacing)) + 1;
    return std::clamp(nodes, 2, kMaxCeilingNodes);
}

// Throws MoveError unless speed is non-negative and finite. Direction comes from
// the sign of the distance, so a negative speed has no meaning.
void requireSpeed(const std::string& name, double speed) {
    if (!(speed >= 0.0) || !std::isfinite(speed)) {
        std::ostringstream msg;
        msg << name << " must be a non-negative, finite speed; got " << speed;
        throw MoveError(msg.str());
    }
}

// ScalarProfile works in magnitudes along the direction of travel; this puts
// the sign back. Zero stays +0 so a reverse move does not print "-0".
double signedValue(double direction, double magnitude) {
    return magnitude == 0.0 ? 0.0 : direction * magnitude;
}

// Runs profile, which covers length, to completion and appends its samples to
// out, placed on the axis at origin + direction * (distance travelled).
// emitStart is false for a profile that continues from out's last sample, which
// already covers its starting point.
void appendSamples(ScalarProfile& profile, double length, bool emitStart, double origin,
                   double direction, std::vector<ScalarSample>& out) {
    if (emitStart) {
        profile.start();
    }
    while (!profile.isFinished()) {
        profile.step();
    }
    // isFinished() is also true once the profiler's step watchdog trips.
    // Returning those samples would hand back a move that silently stops short.
    if (!profile.samples().empty() && profile.samples().back().position < length) {
        throw MoveError("the move did not reach its end before the profiler's 60 s "
                        "watchdog stopped it");
    }
    for (const ScalarSample& s : profile.samples()) {
        out.push_back(ScalarSample{ origin + direction * s.position,
                                    signedValue(direction, s.velocity),
                                    signedValue(direction, s.accel), s.time });
    }
}
} // namespace

std::vector<ScalarSample> generateScalarProfile(
    double distance,
    const ScalarProfileConfig& config,
    double startVel,
    double endVel,
    std::vector<ScalarKeyframe> keyframes
) {
    const double maxVelocity = requireLimit("maxVelocity", config.maxVelocity);
    const double maxAccel = requireLimit("maxAccel", config.maxAccel);
    requirePositiveLimit("dt", config.dt);
    requireSpeed("startVel", startVel);
    requireSpeed("endVel", endVel);
    for (const ScalarKeyframe& kf : keyframes) {
        requireSpeed("keyframe velocity", kf.velocity);
    }
    if (!std::isfinite(distance)) {
        throw MoveError("distance must be finite");
    }

    if (distance == 0.0) {
        return {};
    }
    const double direction = distance < 0.0 ? -1.0 : 1.0;
    const double length = std::fabs(distance);

    const int nodes = ceilingNodeCount(length, maxVelocity, config.dt);

    std::vector<double> ceilingDistances(nodes);
    for (int i = 0; i < nodes; ++i) {
        ceilingDistances[i] = length * static_cast<double>(i) / (nodes - 1);
    }
    // Flat ceiling: with no path there is no curvature to cap speed, only the
    // top speed. Keyframes are folded in by the profiler.
    std::vector<double> ceilingVelocities(nodes, maxVelocity);

    // ScalarProfile sweeps keyframes in order, so they must be sorted by
    // distance.
    std::sort(keyframes.begin(), keyframes.end(),
              [](const ScalarKeyframe& a, const ScalarKeyframe& b) {
                  return a.distance < b.distance;
              });

    ScalarProfile profile(length, std::move(ceilingDistances), std::move(ceilingVelocities),
                          maxAccel, startVel, endVel, config.dt, std::move(keyframes));
    std::vector<ScalarSample> samples;
    appendSamples(profile, length, true, 0.0, direction, samples);
    return samples;
}

std::vector<ScalarSample> generateScalarMoves(
    const ScalarProfileConfig& config,
    double startPosition,
    const std::vector<ScalarMove>& moves
) {
    const double maxVelocity = requireLimit("maxVelocity", config.maxVelocity);
    const double maxAccel = requireLimit("maxAccel", config.maxAccel);
    requirePositiveLimit("dt", config.dt);
    if (!std::isfinite(startPosition)) {
        throw MoveError("startPosition must be finite");
    }

    // Where each move starts, how far it goes and which way. Everything is
    // checked here, before any planning runs.
    struct Leg {
        double origin;
        double length;
        double direction;
    };
    std::vector<Leg> legs;
    legs.reserve(moves.size());
    double position = startPosition;
    for (size_t i = 0; i < moves.size(); ++i) {
        const ScalarMove& move = moves[i];
        const std::string name = "move " + std::to_string(i + 1);
        if (!std::isfinite(move.target)) {
            throw MoveError(name + ": target must be finite");
        }
        const double delta = move.target - position;
        if (delta == 0.0) {
            std::ostringstream msg;
            msg << name << " goes nowhere: it starts at its target " << move.target;
            throw MoveError(msg.str());
        }
        requireSpeed(name + " end velocity", move.endVelocity);
        const double lo = std::min(position, move.target);
        const double hi = std::max(position, move.target);
        for (const MoveKeyframe& kf : move.keyframes) {
            requireSpeed(name + " keyframe velocity", kf.velocity);
            if (!(kf.position >= lo && kf.position <= hi)) {
                std::ostringstream msg;
                msg << name << ": keyframe at " << kf.position << " lies outside the move from "
                    << position << " to " << move.target;
                throw MoveError(msg.str());
            }
        }
        legs.push_back(Leg{ position, std::fabs(delta), delta < 0.0 ? -1.0 : 1.0 });
        position = move.target;
    }
    for (size_t i = 0; i + 1 < legs.size(); ++i) {
        if (legs[i + 1].direction != legs[i].direction && moves[i].endVelocity > 0.0) {
            std::ostringstream msg;
            msg << "move " << i + 1 << " ends at speed " << moves[i].endVelocity << " but move "
                << i + 2 << " reverses direction; a move into a reversal must end at rest";
            throw MoveError(msg.str());
        }
    }

    // Consecutive moves in one direction are profiled as a single run, with each
    // move's end velocity as a ceiling at its join. The backward pass then brakes
    // ahead of a join for whatever the next move demands, and the acceleration
    // limit holds through it. One profile per move could not do that: each would
    // start at a speed its own backward pass may not allow, and drop to that
    // limit in a single step. A run ends where the direction reverses, which is
    // always at rest.
    std::vector<ScalarSample> samples;
    size_t first = 0;
    while (first < legs.size()) {
        size_t last = first;
        while (last + 1 < legs.size() && legs[last + 1].direction == legs[first].direction) {
            ++last;
        }

        std::vector<double> distances;
        std::vector<double> ceiling;
        double runLength = 0.0;
        for (size_t i = first; i <= last; ++i) {
            const Leg& leg = legs[i];

            // Keyframes become distances into this move, in order.
            std::vector<ScalarKeyframe> keyframes;
            keyframes.reserve(moves[i].keyframes.size());
            for (const MoveKeyframe& kf : moves[i].keyframes) {
                keyframes.push_back(
                    ScalarKeyframe{ kf.velocity, leg.direction * (kf.position - leg.origin) });
            }
            std::sort(keyframes.begin(), keyframes.end(),
                      [](const ScalarKeyframe& a, const ScalarKeyframe& b) {
                          return a.distance < b.distance;
                      });

            const int nodes = ceilingNodeCount(leg.length, maxVelocity, config.dt);
            size_t kfIdx = 0;
            for (int j = 0; j < nodes; ++j) {
                const double s = leg.length * (static_cast<double>(j) / (nodes - 1));
                const double cap =
                    std::min(maxVelocity, keyframeVelocityCeiling(s, keyframes, kfIdx));
                if (j == 0 && i != first) {
                    // The join is already the previous move's last node, and has
                    // to satisfy both moves.
                    ceiling.back() = std::min(ceiling.back(), cap);
                    continue;
                }
                distances.push_back(runLength + s);
                ceiling.push_back(cap);
            }
            runLength += leg.length;
            if (i != last) {
                ceiling.back() = std::min(ceiling.back(), moves[i].endVelocity);
            }
        }

        const bool firstRun = samples.empty();
        const double timeAccum = firstRun ? 0.0 : samples.back().time;
        ScalarProfile profile(runLength, std::move(distances), std::move(ceiling), maxAccel, 0.0,
                              moves[last].endVelocity, config.dt, {}, 0.0, timeAccum);
        appendSamples(profile, runLength, firstRun, legs[first].origin, legs[first].direction,
                      samples);
        // origin + direction * runLength can miss the target by a rounding
        // error; the run ends exactly where the caller asked.
        samples.back().position = moves[last].target;
        first = last + 1;
    }
    return samples;
}
