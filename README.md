# vmplib (VEX Motion Profiling Library)

A C++17 library for 2D motion profiling on cubic Bézier paths, aimed at VEX
robotics. Give it a path and a drivetrain, get back a time-parameterised
trajectory: poses, linear velocity and angular velocity, one sample every `dt`.

It implements the methods described in

> *2D Motion Profiling for Competitive Robotics*
> [Read the PDF](https://github.com/SerrialError/latex-papers/blob/main/2dmp.pdf)

Profiles are generated **offline** — on a laptop, ahead of a match — and the
resulting samples are what you ship to the robot. Nothing here needs to run on a
V5 brain.

---

## What it does

- **Cubic Bézier geometry.** Position, derivatives, parametric speed and signed
  curvature anywhere on a segment.
- **Arc-length parameterisation.** 5-point Gauss–Legendre quadrature over
  composite panels for `s(t)`, and a Newton–Raphson inverse for `t(s)`.
- **Two-pass velocity profiling.** The velocity ceiling — top speed, the
  curvature limit `v_max · R / (R + w/2)`, and any keyframes — is sampled up
  front, then swept backward with `v² = v_next² + 2a·Δs`. That is what makes the
  robot brake *on approach* to a corner instead of falling off a cliff at it.
  The forward acceleration pass is applied online, one timestep at a time.
- **Keyframes.** Pin a target speed to a point on the field. The point is
  projected onto the curve in 2D, and speeds between keyframes are interpolated
  in `v²`, which makes each interval a constant-acceleration segment.
- **Multi-segment paths.** Velocity and the timestep grid stay continuous across
  segment joins; a step that overshoots the end of one segment carries its
  leftover arc length into the next.
- **RAMSETE simulation.** Every trajectory also comes back with what a RAMSETE
  follower actually achieves tracking it, which is how you tell whether a plan is
  trackable at all.

---

## Build

Needs GNU Make and a C++17 compiler (`g++` or `clang++`).

```bash
git clone https://github.com/SerrialError/vmplib.git
cd vmplib
make            # builds bin/main
make test       # builds and runs the doctest suite
make clean
```

With [devenv](https://devenv.sh) and [direnv](https://direnv.net):

```bash
direnv allow    # auto-loads the shell on cd, or run `devenv shell`
build           # == make all
run --file examples/path-points.txt
clean           # == make clean
```

---

## Command-line use

```bash
./bin/main --file examples/path-points.txt
```

| Flag | Default | Meaning |
|---|---|---|
| `--file <path>` | *required* | Path file to profile |
| `--out <path>` | `output.txt` | Where to write the result |
| `--format desmos\|code` | `desmos` | Output style |

`--format desmos` emits six lists you can paste straight into Desmos:

| Label | Contents |
|---|---|
| `X` | planned poses, as `(x, y)` |
| `L` | planned linear velocity, as `(t, v)` |
| `A` | planned angular velocity, as `(t, ω)` |
| `X_r`, `L_r`, `A_r` | the same three for the RAMSETE-followed trajectory |

`--format code` emits `P` and `V` as C++ initialiser lists, for pasting into
robot code that replays a fixed trajectory.

### Path file format

Plain text, one block per segment. This is the export format of
[path.jerryio](https://path.jerryio.com), so a file saved from there works
unmodified.

```
#PATH-START Path
#POINTS-START
-0.586, -0.410          <- four control points per cubic Bezier segment
-0.586, -0.201
-0.997,  0.335
-0.997,  0.544
#VELOCITIES-START
-0.700,  0.100, 0.3     <- x, y, target speed (m/s)
#PATH.JERRYIO-DATA {...}
```

`#VELOCITIES-START` may be empty. Each keyframe is an `(x, y)` point on the
field plus the speed you want there; the point is projected onto the curve
rather than matched by `x` alone, so paths that double back work correctly.

Units are metres and metres per second throughout. If your editor works in
centimetres, scale before feeding the file in.

---

## Library use

```cpp
#include "motion-profiler.hpp"
#include "file-parser.hpp"

std::vector<std::vector<Point>> controlPoints;
std::vector<std::vector<KeyframeVelocitiesXandY>> keyframes;
loadPaths("path.txt", controlPoints, keyframes);

ProfileConfig config;
config.maxVelocity = 1.89f;   // m/s
config.maxAccel    = 4.12f;   // m/s^2
config.trackWidth  = 0.295f;  // m
config.dt          = 0.01f;   // s

const Trajectory traj = generateTrajectory(controlPoints, keyframes, true, config);
```

`controlPoints` and `keyframes` are indexed in parallel — one entry each per
segment — and `keyframes` is ignored entirely when the third argument is
`false`. A segment with no keyframes gets an empty list, not a placeholder.

`Trajectory` holds four vectors, each with one outer entry per segment:

```cpp
struct Trajectory {
    std::vector<std::vector<Pose>> poses;                       // planned
    std::vector<std::vector<VelocityLayout>> velocities;        // planned
    std::vector<std::vector<Pose>> followedPoses;               // RAMSETE
    std::vector<std::vector<VelocityLayout>> followedVelocities;
};
```

`VelocityLayout` is `{ linear, angular, time }`, with `time` accumulating across
segments so the whole trajectory shares one clock.

### Driving a differential drivetrain

Convert each sample to wheel speeds with the track width `w`:

```cpp
const float left  = v.linear - (v.angular * trackWidth) / 2.0f;
const float right = v.linear + (v.angular * trackWidth) / 2.0f;
```

That is open loop — it assumes the robot ends up where the plan says. Close the
loop with odometry and a tracking controller; `RamseteFollower` in
`include/ramsete.hpp` is the one this library simulates against.

### Tuning

`ProfileConfig` defaults to the robot this project was originally tuned against.
Override the fields rather than editing the source.

| Field | Default | Notes |
|---|---|---|
| `maxVelocity` | 1.8885 m/s | Measured free speed, not the theoretical one |
| `maxAccel` | 4.1220 m/s² | Used for both accelerating and braking |
| `trackWidth` | 0.2951 m | Sets both the curvature limit and the wheel-speed split |
| `ramseteB` | 2.0 m⁻² | Larger converges harder |
| `ramseteZeta` | 0.7 | Damping ratio, in (0, 1) |
| `dt` | 0.01 s | Sample period; must match your control loop |

### Errors

`generateTrajectory` throws `KeyframeError` (declared in `bezier.hpp`, derives
from `std::runtime_error`) when a keyframe cannot be placed — either it sits too
far off the curve to be meaningful, or it is out of order along the path. It
throws rather than quietly snapping the keyframe somewhere arbitrary, because
generation is offline and a wrong trajectory is worse than a failed run.

---

## A caveat on the commanded velocity

Successive samples can differ by up to `2·a·dt`, not `a·dt`. Each sample
commands the speed at the *start* of its step while the step advances by `v·dt`,
so on a braking ramp `Δv = v − √(v² − 2a·v·dt)`. That tends to `a·dt` at speed
but peaks at exactly `2a·dt` when `v = 2a·dt`. It is a discretization artifact
rather than a planning error, and it shrinks with `dt`.

---

## Known limitations

- A segment's exit velocity is taken from its last keyframe regardless of where
  that keyframe sits on the segment, so a path whose final segment has an early
  keyframe will not come to rest
  ([#21](https://github.com/SerrialError/vmplib/issues/21)).
- Acceleration is modelled as a constant, so the profile is optimistic at high
  speed where a real motor makes less torque.

## Future plans

- PROS module for on-robot playback
- CLI for path import/export
- Tighter integration with path.jerryio and other route planners

Contributions, issues, and pull requests are welcome.
