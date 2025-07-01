#include "motion-profiler.hpp"
#include "motion-utils.hpp"       // for sFunction, curvature, findXandY, findTForS, wrapAngle, sinc
#include "motion-profiling.hpp"   // TrapezoidalProfile
#include "ramsete.hpp"            // RamseteFollower
#include "printer.hpp"            // Printer::printPoseVector / printVelocityVector
#include <cmath>
#include <vector>
#include <string>

// ----------------------------------------------------------------------------
// Constants (same values as your original printVels() used)
constexpr float   MAX_VELOCITY = 1.94503855166f;  // m/s
constexpr float   MAX_ACCEL    = 6.67663722193f;  // m/s²
constexpr float   TRACK_WIDTH  = 0.288925f;       // meters
constexpr float   RAMSETE_B    = 2.0f;
constexpr float   RAMSETE_ZETA = 0.7f;
constexpr float   DT           = 0.01f;           // 10 ms timestep

using namespace MotionUtils; // for sFunction, curvature, findXandY, findTForS, wrapAngle, sinc

void printVels(
    const std::vector<Point>& controlPoints,
    const std::vector<KeyframeVelocitiesXandY>& keyFrameVelocityInitList,
    bool useKeyFrames
) {
    // 1) Convert (x,y,velocity) keyframes → (time, scalar‐velocity) keyframes,
    //    only if the user requested key-frame limiting.
    std::vector<KeyframeVelocities> keyframes;
    if (useKeyFrames && !keyFrameVelocityInitList.empty()) {
        keyframes = convertToTFrame(controlPoints, keyFrameVelocityInitList);
    }

    // 2) Determine initial and exit velocity
    float initialVel = (useKeyFrames && !keyframes.empty())
                       ? keyframes.front().velocity
                       : 0.0f;
    float exitVel = (useKeyFrames && !keyframes.empty())
                    ? keyframes.back().velocity
                    : 0.0f;

    // 3) Compute total spline length = sFunction(…, t=1)
    float totalLength = sFunction(controlPoints, 1.0f);

    // 4) Compute deceleration distance exactly as in your original code:
    //    decelDist = totalLength − [ (exitVel² − maxVel²) / (−2·maxAccel) ]
    float decelDist = totalLength 
                    - ((std::pow(exitVel, 2.0f) - std::pow(MAX_VELOCITY, 2.0f))
                       / (-2.0f * MAX_ACCEL));
    
    float timeAccum = .0f;
    // 5) Build and run the trapezoidal profile
    TrapezoidalProfile profiler(
        controlPoints,
        MAX_VELOCITY,
        MAX_ACCEL,
        decelDist,
	timeAccum,
        initialVel,
        exitVel,
        keyframes,
        useKeyFrames,
        DT
    );

    profiler.start();
    while (!profiler.isFinished()) {
        profiler.step();
    }

    // 6) Print the open-loop (“nominal”) path:
    Printer::printPoseVector(    "X = ",  profiler.getPoses()           );
    Printer::printVelocityVector("L = ",  profiler.getVelocities(), "linear"  );
    Printer::printVelocityVector("A = ",  profiler.getVelocities(), "angular" );

    // 7) Build & run the RAMSETE follower in reverse mode (as your old code did)
    RamseteFollower ramser(
        TRACK_WIDTH,
        RAMSETE_B,
        RAMSETE_ZETA,
        DT
    );
    ramser.initialize(profiler.getPoses(), profiler.getVelocities(), /*reverse=*/true);

    while (!ramser.isFinished()) {
        ramser.step();
    }

    // 8) Print the closed-loop (“RAMSETE‐executed”) path:
    Printer::printPoseVector(    "X_r = ",  ramser.getExecutedPoses()           );
    Printer::printVelocityVector("L_r = ",  ramser.getExecutedVelocities(), "linear"  );
    Printer::printVelocityVector("A_r = ",  ramser.getExecutedVelocities(), "angular" );
}
