#include "doctest.h"
#include "file-parser.hpp"

#include <cstdio>
#include <fstream>
#include <unistd.h>
#include <string>
#include <vector>

namespace {

// Writes contents to a scratch file and removes it when the test is done, so a
// parser test does not depend on anything in the repo or leave litter behind.
struct ScratchFile {
    std::string path;
    explicit ScratchFile(const std::string& contents)
        : path(std::string("/tmp/vmplib_parser_test_") + std::to_string(::getpid()) + "_" +
               std::to_string(counter()++) + ".txt") {
        std::ofstream out(path);
        out << contents;
    }
    ~ScratchFile() { std::remove(path.c_str()); }
    static int& counter() {
        static int n = 0;
        return n;
    }
};

std::pair<std::vector<std::vector<Point>>, std::vector<std::vector<KeyframeVelocitiesXandY>>>
load(const std::string& path) {
    std::vector<std::vector<Point>> pts;
    std::vector<std::vector<KeyframeVelocitiesXandY>> vels;
    loadPaths(path, pts, vels);
    return {pts, vels};
}

} // namespace

TEST_CASE("loadPaths throws when the file cannot be opened") {
    // A missing file used to be a cerr line and a normal return, leaving the
    // caller with an empty trajectory and exit status 0.
    CHECK_THROWS_AS(load("/tmp/vmplib_does_not_exist_zzz.txt"), FileParseError);
}

TEST_CASE("loadPaths throws on a malformed point line") {
    ScratchFile file(
        "#PATH-START Path\n"
        "#POINTS-START\n"
        "0.0, 0.0\n"
        "this is not a point\n"
        "#VELOCITIES-START\n");
    CHECK_THROWS_AS(load(file.path), FileParseError);
}

TEST_CASE("loadPaths throws on a malformed velocity line") {
    ScratchFile file(
        "#PATH-START Path\n"
        "#POINTS-START\n"
        "0.0, 0.0\n"
        "1.0, 0.0\n"
        "2.0, 0.0\n"
        "3.0, 0.0\n"
        "#VELOCITIES-START\n"
        "0.0, 0.0, oops\n");
    CHECK_THROWS_AS(load(file.path), FileParseError);
}

TEST_CASE("loadPaths reads a well-formed single-segment path") {
    ScratchFile file(
        "#PATH-START Path\n"
        "#POINTS-START\n"
        "0.0, 0.0\n"
        "1.0, 0.0\n"
        "2.0, 0.0\n"
        "3.0, 0.0\n"
        "#VELOCITIES-START\n");

    std::vector<std::vector<Point>> pts;
    std::vector<std::vector<KeyframeVelocitiesXandY>> vels;
    CHECK_NOTHROW(loadPaths(file.path, pts, vels));

    REQUIRE(pts.size() == 1);
    REQUIRE(pts[0].size() == 4);
    CHECK(pts[0][3].x == doctest::Approx(3.0));
    REQUIRE(vels.size() == 1);
    CHECK(vels[0].empty());
}
