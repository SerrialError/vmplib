#include "doctest.h"
#include "file-parser.hpp"
#include "mechanism-file.hpp"
#include "scratch-file.hpp"

#include <string>

namespace {

// Checks that contents fails to load with message, which follows the file's
// path in the error.
void checkLoadError(const std::string& contents, const std::string& message) {
    CAPTURE(contents);
    ScratchFile file(contents);
    std::string error;
    try {
        loadMoves(file.path);
    } catch (const FileParseError& e) {
        error = e.what();
    }
    CHECK(error == file.path + message);
}

// The same, for a velocity file.
void checkVelocityLoadError(const std::string& contents, const std::string& message) {
    CAPTURE(contents);
    ScratchFile file(contents);
    std::string error;
    try {
        loadVelocityTargets(file.path);
    } catch (const FileParseError& e) {
        error = e.what();
    }
    CHECK(error == file.path + message);
}

} // namespace

TEST_CASE("loadMoves reads moves, keyframes and comments") {
    ScratchFile file(
        "// Raise the lift, slowing through the middle, then lower it.\n"
        "#MOVE-START raise\n"
        "#FROM 0.1\n"
        "#TO 0.8          // top of travel\n"
        "#END-VELOCITY 0\n"
        "#KEYFRAMES-START\n"
        "  0.1, 1.2\n"
        "  0.4,0.3\n"
        "\n"
        "  0.8 , 1.2\n"
        "#MOVE-START lower\n"
        "#FROM 0.8\n"
        "#TO 0.2\n"
        "#MOVE-START\n"
        "#TO 0.5\n"
        "#END-VELOCITY 0.25\n");

    const MoveFile moves = loadMoves(file.path);
    CHECK(moves.startPosition == doctest::Approx(0.1));
    REQUIRE(moves.moves.size() == 3);

    CHECK(moves.moves[0].target == doctest::Approx(0.8));
    CHECK(moves.moves[0].endVelocity == doctest::Approx(0.0));
    REQUIRE(moves.moves[0].keyframes.size() == 3);
    CHECK(moves.moves[0].keyframes[1].position == doctest::Approx(0.4));
    CHECK(moves.moves[0].keyframes[1].velocity == doctest::Approx(0.3));

    CHECK(moves.moves[1].target == doctest::Approx(0.2));
    CHECK(moves.moves[1].keyframes.empty());

    CHECK(moves.moves[2].target == doctest::Approx(0.5));
    CHECK(moves.moves[2].endVelocity == doctest::Approx(0.25));
}

TEST_CASE("a move file without #FROM starts at zero") {
    ScratchFile file("#MOVE-START\n#TO -1.5\n");
    const MoveFile moves = loadMoves(file.path);
    CHECK(moves.startPosition == 0.0);
    REQUIRE(moves.moves.size() == 1);
    CHECK(moves.moves[0].target == doctest::Approx(-1.5));
}

TEST_CASE("a loaded move file profiles end to end") {
    ScratchFile file(
        "#MOVE-START raise\n#TO 0.8\n"
        "#KEYFRAMES-START\n0.0, 1.2\n0.4, 0.3\n0.8, 1.2\n"
        "#MOVE-START lower\n#TO 0.2\n");
    const MoveFile moves = loadMoves(file.path);
    const auto samples =
        generateScalarMoves(ScalarProfileConfig{1.2, 3.0}, moves.startPosition, moves.moves);
    REQUIRE(!samples.empty());
    CHECK(samples.back().position == 0.2);
}

TEST_CASE("loadMoves reports malformed files with their line number") {
    checkLoadError("#MOVE-START\n#TO 0.8\n#TO 0.9\n", ":3: #TO is given twice in this move");
    checkLoadError("#MOVE-START up\n#END-VELOCITY 0.2\n", ":1: move 'up' has no #TO");
    checkLoadError("#MOVE-START up\n#TO 0.8\n#MOVE-START down\n#FROM 0.7\n#TO 0.0\n",
                   ":3: move 'down' starts #FROM 0.7 but the move before it ends at 0.8");
    checkLoadError("#MOVE-START\n#TO high\n", ":2: #TO expects a number, got 'high'");
    checkLoadError("#MOVE-START\n#TO inf\n", ":2: #TO expects a number, got 'inf'");
    checkLoadError("#MOVE-START\n#TO 0.8\n#SPEED 3\n", ":3: unknown marker #SPEED");
    checkLoadError("#TO 0.8\n", ":1: #TO must come after a #MOVE-START");
    checkLoadError("#MOVE-START\n#TO 0.8\n0.4, 0.3\n",
                   ":3: expected a #MARKER line, got '0.4, 0.3'");
    checkLoadError("#MOVE-START\n#TO 0.8\n#KEYFRAMES-START now\n",
                   ":3: #KEYFRAMES-START takes no value");
}

TEST_CASE("loadMoves rejects a keyframe that is not exactly 'position, speed'") {
    for (const std::string bad : {"0.4 0.3", "0.4, 0.3, 1", "0.4,", ", 0.3", "0.4, fast"}) {
        checkLoadError("#MOVE-START\n#TO 0.8\n#KEYFRAMES-START\n" + bad + "\n",
                       ":4: expected a keyframe as 'position, speed', got '" + bad + "'");
    }
}

TEST_CASE("loadMoves rejects a file with no moves, or one it cannot open") {
    checkLoadError("// nothing here\n\n", ": no moves; each move begins with #MOVE-START");
    CHECK_THROWS_AS(loadMoves("/tmp/vmplib_does_not_exist_zzz.txt"), FileParseError);
}

TEST_CASE("loadVelocityTargets reads targets, holds and comments") {
    ScratchFile file(
        "// Spin up, hold for the shot, spin down.\n"
        "#TARGET-START spin-up\n"
        "#VELOCITY 400    // rad/s\n"
        "#HOLD 1.5\n"
        "\n"
        "#TARGET-START reverse\n"
        "#VELOCITY -120\n"
        "#TARGET-START\n"
        "#HOLD 0.25\n"
        "#VELOCITY 0\n");

    const std::vector<VelocityTarget> targets = loadVelocityTargets(file.path);
    REQUIRE(targets.size() == 3);
    CHECK(targets[0].velocity == doctest::Approx(400.0));
    CHECK(targets[0].hold == doctest::Approx(1.5));
    CHECK(targets[1].velocity == doctest::Approx(-120.0));
    CHECK(targets[1].hold == 0.0);
    CHECK(targets[2].velocity == 0.0);
    CHECK(targets[2].hold == doctest::Approx(0.25));
}

TEST_CASE("a loaded velocity file profiles end to end") {
    ScratchFile file("#TARGET-START\n#VELOCITY 400\n#HOLD 0.1\n#TARGET-START\n#VELOCITY 0\n");
    const auto samples = generateVelocityProfile(VelocityProfileConfig{std::nullopt, 800.0},
                                                 loadVelocityTargets(file.path));
    REQUIRE(!samples.empty());
    CHECK(samples.back().velocity == 0.0);
}

TEST_CASE("loadVelocityTargets reports malformed files with their line number") {
    checkVelocityLoadError("#TARGET-START idle\n#HOLD 1\n", ":1: target 'idle' has no #VELOCITY");
    checkVelocityLoadError("#TARGET-START\n#VELOCITY 1\n#HOLD 1\n#HOLD 2\n",
                           ":4: #HOLD is given twice in this target");
    checkVelocityLoadError("#TARGET-START\n#VELOCITY fast\n",
                           ":2: #VELOCITY expects a number, got 'fast'");
    checkVelocityLoadError("#TARGET-START\n#TO 0.8\n", ":2: unknown marker #TO");
    checkVelocityLoadError("#VELOCITY 1\n", ":1: #VELOCITY must come after a #TARGET-START");
    checkVelocityLoadError("#TARGET-START\n#VELOCITY 1\n400\n",
                           ":3: expected a #MARKER line, got '400'");
    checkVelocityLoadError("// empty\n", ": no targets; each target begins with #TARGET-START");
}
