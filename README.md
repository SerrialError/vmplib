# vmplib (VEX Motion Profiling Library)

A C++17 library for 2D motion profiling on cubic Bézier paths, aimed at VEX
robotics. Give it a path and a drivetrain, get back a time-parameterised
trajectory: poses, linear velocity and angular velocity, one sample every `dt`.

It implements the methods described in

> *2D Motion Profiling for Competitive Robotics*
> [Read the PDF](https://github.com/SerrialError/latex-papers/blob/main/2dmp.pdf)

Profiles are generated **offline** which can then be exported to your realtime code you run on your brain.

---

## What it does

- **Cubic Bézier geometry.** Position, derivatives, parametric speed and signed
  curvature anywhere on a segment.
- **Arc-length parameterisation.** 5-point Gauss–Legendre quadrature over
  composite panels for `s(t)`, and a Newton–Raphson inverse for `t(s)`.
- **Two-pass velocity profiling.** The velocity ceiling — top speed, the
  curvature limit , and any keyframes — is sampled up
  front, then swept backward.
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

Units are default SI units throughout.

---

### Tuning

`ProfileConfig` defaults to the robot this project was originally tuned against.
Override the fields rather than editing the source.

| Field | Default |
|---|---|
| `maxVelocity` | 1.8885 m/s |
| `maxAccel` | 4.1220 m/s² |
| `trackWidth` | 0.2951 m |
| `ramseteB` | 2.0 m⁻² |
| `ramseteZeta` | 0.7 |
| `dt` | 0.01 s |

---

## 1D profiling

The same velocity profiler drives a single axis — a straight drive, a turret, a
lift — without any of the 2D path machinery. `generateScalarProfile`
(`scalar-profiler.hpp`) takes a distance and motion limits and returns one
`(position, velocity, accel, time)` sample every `dt`:

```cpp
#include "scalar-profiler.hpp"

const ScalarProfileConfig config{ /*maxVelocity=*/1.5, /*maxAccel=*/3.0, /*dt=*/0.01 };

// Optional: pin a velocity to a distance along the move (need not be pre-sorted).
const std::vector<ScalarKeyframe> keyframes = {{1.5, 0.0}, {0.4, 1.0}, {1.5, 2.0}};

const std::vector<ScalarSample> samples =
    generateScalarProfile(/*distance=*/2.0, config, /*startVel=*/0.0, /*endVel=*/0.0,
                          keyframes);
```

It is **unit-agnostic**: distance, velocity, and acceleration may be metres,
inches, or radians, as long as they are consistent. Gearing and motor
conversions stay in your robot code. See `examples/scalar_profile_demo.cpp`:

```bash
make example
./bin/scalar-profile-demo
```

The 2D `generateTrajectory` and this 1D API share the same underlying
`ScalarProfile` core, so both land exactly on their exit velocity and never
exceed the acceleration limit — including on the final step.

---

## Future plans

- CLI for path import/export
- Tighter integration with path.jerryio and other route planners

Contributions, issues, and pull requests are welcome.
