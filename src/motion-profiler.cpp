#include "motion-profiler.hpp"
#include "motion-utils.hpp"       // for sFunction, curvature, findXandY, findTForS, wrapAngle, sinc
#include "motion-profiling.hpp"   // TrapezoidalProfile
#include "ramsete.hpp"            // RamseteFollower
#include "printer.hpp"            // Printer::printPoseVector / printVelocityVector
#include <algorithm>
#include <cmath>
#include <vector>
#include <string>

// ----------------------------------------------------------------------------
// Constants (same values as your original printVels() used)
constexpr float   MAX_VELOCITY = 1.8885160604f;  // m/s
// (min of 75.858991, 74.351026, 80.215332, 76.235982, 75.314448, 78.351321) * 1 * 0.0254 
constexpr float   MAX_ACCEL    = 4.12203073382f;  // m/s²
// (min of 1321.05645016, 1725.94898788, 5072.71894467, 3516.54633835, 1360.24316837, 5810.0992236) * 1 * 0.0254 = 3.5548338341 NOT REALISTIC SO 5.15253841728 * 0.8 = 4.12203073382
constexpr float   TRACK_WIDTH  = 0.29508135f;       // meters
constexpr float   RAMSETE_B    = 2.0f;
constexpr float   RAMSETE_ZETA = 0.7f;
constexpr float   DT           = 0.01f;           // 10 ms timestep
// A keyframe within this much of t = 0 counts as sitting on the segment join,
// where the only room to brake for it is in the segment before.
constexpr float   JUNCTION_KEYFRAME_T_TOL = 1e-3f;
bool testing = true;

using namespace MotionUtils; // for sFunction, curvature, findXandY, findTForS, wrapAngle, sinc

void printVels(
    const std::vector<std::vector<Point>>& controlPoints,
    const std::vector<std::vector<KeyframeVelocitiesXandY>>& keyFrameVelocityInitList,
    bool useKeyFrames
) {
    float timeAccum = 0.0f;
    std::vector<std::vector<Pose>> Poses;
    std::vector<std::vector<VelocityLayout>> Velocities;
    std::vector<std::vector<Pose>> RamsetePoses;
    std::vector<std::vector<VelocityLayout>> RamseteVelocities;
    // 1) Convert every segment's (x,y,velocity) keyframes to (velocity, t) up
    //    front. A segment's exit velocity depends on the *next* segment's
    //    keyframes, which cannot be resolved one segment at a time.
    std::vector<std::vector<KeyframeVelocities>> keyframesPerSegment(controlPoints.size());
    if (useKeyFrames) {
        const size_t n = std::min(controlPoints.size(), keyFrameVelocityInitList.size());
        for (size_t i = 0; i < n; ++i) {
            keyframesPerSegment[i] = convertToTFrame(controlPoints[i], keyFrameVelocityInitList[i]);
        }
    }

    float carryOverArcLength = 0.0f;
    bool havePreviousSegment = false;

    for (size_t i = 0; i < controlPoints.size(); ++i) {

      const std::vector<KeyframeVelocities>& keyframes = keyframesPerSegment[i];

      // 2) Determine initial and exit velocity.
      //    Velocity is continuous across a junction, so a segment starts at
      //    whatever the previous one actually ended at. Only the first segment
      //    is free to take its start velocity from a keyframe.
      float initialVel = 0.0f;
      if (havePreviousSegment) {
          initialVel = Velocities.back().back().linear;
      } else if (!keyframes.empty()) {
          initialVel = keyframes.front().velocity;
      }

      float exitVel = keyframes.empty() ? 0.0f : keyframes.back().velocity;

      //    A keyframe sitting on the first point of the next segment has no
      //    room to be met inside that segment, so it is really a constraint on
      //    this segment's exit velocity. Braking for it has to start here.
      if (i + 1 < keyframesPerSegment.size()) {
          const std::vector<KeyframeVelocities>& next = keyframesPerSegment[i + 1];
          if (!next.empty() && next.front().t <= JUNCTION_KEYFRAME_T_TOL) {
              exitVel = next.front().velocity;
          }
      }

      // 3) Build and run the trapezoidal profile. Deceleration comes from the
      //    backward pass inside the profile, so no braking distance is needed
      //    here. carryOverArcLength resumes where the previous segment's last
      //    timestep overshot the join.
      TrapezoidalProfile profiler(
          controlPoints[i],
          MAX_VELOCITY,
          MAX_ACCEL,
          TRACK_WIDTH,
	  	  timeAccum,
          initialVel,
          exitVel,
          keyframes,
          useKeyFrames,
          DT,
          carryOverArcLength
      );

      // Only the first segment emits a sample at its start pose. For the rest
      // that pose is the join, already covered by the previous segment's final
      // sample, and re-emitting it would duplicate a timestamp.
      if (!havePreviousSegment) {
          profiler.start();
      }
      while (!profiler.isFinished()) {
          profiler.step();
      }
      carryOverArcLength = profiler.overshootArcLength();
      if (profiler.getVelocities().empty()) {
          // The carry-over covered this segment whole, so it produced no
          // samples. The remainder rolls on to the next one.
          continue;
      }
      havePreviousSegment = true;
      Poses.push_back(profiler.getPoses());
      Velocities.push_back(profiler.getVelocities());

      // 7) Build & run the RAMSETE follower
      RamseteFollower ramser(
          profiler.getPoses(), 
	  	  profiler.getVelocities(),
	  	  TRACK_WIDTH,
          RAMSETE_B,
          RAMSETE_ZETA,
	  	  timeAccum,
          DT,
	  	  false
      );

      while (!ramser.isFinished()) {
          ramser.step();
      }

      RamsetePoses.push_back(ramser.getExecutedPoses());
      RamseteVelocities.push_back(ramser.getExecutedVelocities());
      timeAccum = profiler.getVelocities().back().time;
    }
	if (testing) {
		// 6) Print the open-loop (“nominal”) path:
		Printer::printPoseVectorDesmos(    "X = ",  Poses           );
		Printer::printVelocityVectorDesmos("L = ",  Velocities, "linear"  );
		Printer::printVelocityVectorDesmos("A = ",  Velocities, "angular" );
		// 8) Print the closed-loop (“RAMSETE‐executed”) path:
		Printer::printPoseVectorDesmos(    "X_r = ",  RamsetePoses           );
		Printer::printVelocityVectorDesmos("L_r = ",  RamseteVelocities, "linear"  );     
		Printer::printVelocityVectorDesmos("A_r = ",  RamseteVelocities, "angular" );
	}
	else {
		// 6) Print the open-loop (“nominal”) path:
		Printer::printPoseVectorCode(    "P =",  Poses           );
		Printer::printVelocityVectorCode("V =",  Velocities);
	}

}
