# vmplib (VEX Motion Profiling Library)

An offline 1D and 2D Motion Profiler for drivetrains and other mechanisms.

The theory behind the program was described and developed in a paper I made

> *2D Motion Profiling for Competitive Robotics*
> [Read the PDF](https://github.com/SerrialError/latex-papers/blob/main/2dmp.pdf)

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
run path --file examples/path-points.txt --max-vel 1.8885 --max-accel 4.1220 --track-width 0.2951
clean           # == make clean
```

---

## Command-line use

`bin/main` takes a mode, then that mode's flags. `./bin/main --help` lists them.

### `path`: a differential drive along a Bézier path

```bash
./bin/main path --file examples/path-points.txt --max-vel 1.8885 --max-accel 4.1220 --track-width 0.2951
```

Leaving the mode off (`./bin/main --file ...`) also runs `path`.

| Flag | Default | Meaning |
|---|---|---|
| `--file <path>` | *required* | Path file to profile |
| `--max-vel <m/s>` | *required* | Top linear speed |
| `--max-accel <m/s²>` | *required* | Acceleration limit |
| `--track-width <m>` | *required* | Distance between the left and right wheels |
| `--dt <s>` | `0.01` | Timestep |
| `--out <path>` | `output.txt` | Where to write the result |
| `--format desmos\|code\|rust` | `desmos` | Output style |

`--format desmos` emits six lists you can paste straight into Desmos:

| Label | Contents |
|---|---|
| `X` | planned poses, as `(x, y)` |
| `L` | planned linear velocity, as `(t, v)` |
| `A` | planned angular velocity, as `(t, ω)` |
| `X_r`, `L_r`, `A_r` | the same three for the RAMSETE-followed trajectory |

`--format code` emits `P` and `V` as C++ initialiser lists, for pasting into
robot code that replays a fixed trajectory.

`--format rust` emits a Rust module, for dropping into a robot crate with `mod`.
It declares a `DriveSample` struct and a `SAMPLES` slice holding one per
timestep, all `f64` at full precision:

| Field | Contents |
|---|---|
| `time` | s |
| `linear_velocity` | the robot's speed, m/s |
| `angular_velocity` | the robot's turn rate, rad/s, counterclockwise positive |
| `left_velocity`, `right_velocity` | each side's speed, `v ∓ ω·w/2`, m/s |
| `left_accel`, `right_accel` | each side's acceleration, m/s² |

A sample's accelerations are the ones that carry it to the next sample, so a
controller that holds a sample for one timestep feeds them forward as they are.
The last sample's are 0. From C++, `driveSamples(traj, config)` in
`drive-samples.hpp` returns the same samples.

#### Path file format

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

### `linear`: a lift, arm, turret or straight drive

```bash
./bin/main linear --file examples/lift.txt --max-vel 1.2 --max-accel 3.0
```

| Flag | Default | Meaning |
|---|---|---|
| `--file <path>` | *required* | Move file to profile |
| `--max-vel <units/s>` | *required* | Top speed |
| `--max-accel <units/s²>` | *required* | Acceleration limit |
| `--dt <s>` | `0.01` | Timestep |
| `--out <path>` | `output.txt` | Where to write the result |
| `--format desmos\|code\|rust` | `desmos` | Output style |

Units are whatever the move file uses, as long as the limits match: metres for
a lift, radians for an arm or turret. Gearing and motor conversions stay in your
robot code.

`--format desmos` emits `P`, `V` and `A` (position, velocity and acceleration),
each as `(t, value)`. `--format code` emits `S`, a C++ initialiser list with one
`{position, velocity, accel}` per timestep.

`--format rust` emits a Rust module with a `MotionSample { time, velocity, accel }`
struct and a `SAMPLES` slice holding one per timestep, as `f64`. Each sample's
`accel` is the one that carries it to the next sample, and the last sample's is 0.

#### Move file format

```
// Everything after // is a comment.
#MOVE-START raise      <- begins a move; the name is optional
#FROM 0.0              <- optional: the start position (default 0); later
                          moves' #FROM must match the previous #TO
#TO 0.8                <- required: the position to finish at
#END-VELOCITY 0        <- optional speed at #TO, default 0
#KEYFRAMES-START       <- optional; then one "position, speed" per line
0.0, 1.2
0.4, 0.3
0.8, 1.2
#MOVE-START lower
#TO 0.2
```

Positions are absolute, so each move's direction comes from whether `#TO` is
above or below where it starts. Consecutive moves in the same direction pass
through each join at up to that move's `#END-VELOCITY`, braking early if the
next move needs it. A move followed by a reversal has to end at rest.

From C++:

```cpp
#include "mechanism-file.hpp"

const MoveFile file = loadMoves("examples/lift.txt");
const std::vector<ScalarSample> samples =
    generateScalarMoves(ScalarProfileConfig{/*maxVelocity=*/1.2, /*maxAccel=*/3.0},
                        file.startPosition, file.moves);
```

### `velocity`: a flywheel, roller or intake

```bash
./bin/main velocity --file examples/flywheel.txt --max-accel 800 --max-vel 450
```

| Flag | Default | Meaning |
|---|---|---|
| `--file <path>` | *required* | Velocity file to profile |
| `--max-accel <units/s²>` | *required* | Acceleration limit |
| `--max-vel <units/s>` | unset | If set, a target faster than this is an error |
| `--dt <s>` | `0.01` | Timestep |
| `--out <path>` | `output.txt` | Where to write the result |
| `--format desmos\|code\|rust` | `desmos` | Output style |

The mechanism starts at rest. It ramps to each target in turn at the
acceleration limit, landing exactly on it, then holds it for `#HOLD` seconds,
rounded up to a whole timestep. Velocities are signed, so a roller can run
backwards.

`--format desmos` emits `V` and `A`, each as `(t, value)`. `--format code` emits
`S`, a C++ initialiser list with one `{velocity, accel}` per timestep.
`--format rust` emits the same `MotionSample` module as `linear`.

#### Velocity file format

```
#TARGET-START spin-up  <- begins a target; the name is optional
#VELOCITY 400          <- required: the velocity to ramp to
#HOLD 1.5              <- optional seconds to hold it once reached, default 0
#TARGET-START spin-down
#VELOCITY 0
```

From C++:

```cpp
#include "mechanism-file.hpp"

const std::vector<VelocitySample> samples = generateVelocityProfile(
    VelocityProfileConfig{/*maxVelocity=*/450.0, /*maxAccel=*/800.0},
    loadVelocityTargets("examples/flywheel.txt"));
```

---

### Tuning

The limits that describe your robot have no defaults. `generateTrajectory`
throws `ConfigError` if one is unset or is not a positive, finite number.

| Field | Default |
|---|---|
| `maxVelocity` | *required* (m/s) |
| `maxAccel` | *required* (m/s²) |
| `trackWidth` | *required* (m) |
| `ramseteB` | 2.0 m⁻² |
| `ramseteZeta` | 0.7 |
| `dt` | 0.01 s |

---

## Future plans

- Tighter integration with path.jerryio

Contributions, issues, and pull requests are welcome.
