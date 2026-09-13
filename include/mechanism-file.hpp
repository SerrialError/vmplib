#pragma once

#include <string>
#include <vector>
#include "scalar-profiler.hpp"

// Input files describing what a mechanism should do, written as #MARKER blocks
// in the style of the path.jerryio export the path parser reads. Whitespace
// around values is ignored, as is everything from "//" to the end of a line.

// The moves in a move file, ready for generateScalarMoves.
struct MoveFile {
    double startPosition = 0.0;
    std::vector<ScalarMove> moves;
};

// Reads a move file:
//
//   #MOVE-START raise      <- begins a move; the name is optional
//   #FROM 0.0              <- optional. On the first move it sets the start
//                             position (default 0); on later moves it must
//                             match the previous move's #TO
//   #TO 0.8                <- required: the position to finish at
//   #END-VELOCITY 0        <- optional speed at #TO, default 0
//   #KEYFRAMES-START       <- optional; then one "position, speed" per line
//   0.4, 0.3
//   #MOVE-START lower
//   #TO 0.2
//
// Throws FileParseError (file-parser.hpp), tagged with the file and line, if
// the file cannot be opened, holds no moves, or has a line it cannot make sense
// of. Whether the moves can actually be profiled -- speeds, keyframe placement,
// reversals -- is checked by generateScalarMoves.
MoveFile loadMoves(const std::string& filename);
