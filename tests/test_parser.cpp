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

TEST_CASE("loadPaths splits one 3N+1 spline block into N cubic segments") {
    // A path.jerryio spline stores 3N+1 shared-endpoint control points in a
    // single #POINTS-START block. Seven points is two cubics that share their
    // middle point; the old parser kept only the first four and dropped the
    // rest.
    ScratchFile file(
        "#PATH-START Path\n"
        "#POINTS-START\n"
        "0.0, 0.0\n"
        "1.0, 0.0\n"
        "2.0, 0.0\n"
        "3.0, 0.0\n"
        "4.0, 1.0\n"
        "5.0, 2.0\n"
        "6.0, 3.0\n"
        "#VELOCITIES-START\n");

    std::vector<std::vector<Point>> pts;
    std::vector<std::vector<KeyframeVelocitiesXandY>> vels;
    CHECK_NOTHROW(loadPaths(file.path, pts, vels));

    REQUIRE(pts.size() == 2);
    REQUIRE(pts[0].size() == 4);
    REQUIRE(pts[1].size() == 4);
    // The two cubics share the middle control point.
    CHECK(pts[0][0].x == doctest::Approx(0.0));
    CHECK(pts[0][3].x == doctest::Approx(3.0));
    CHECK(pts[1][0].x == doctest::Approx(3.0));
    CHECK(pts[1][0].y == doctest::Approx(pts[0][3].y));
    CHECK(pts[1][3].x == doctest::Approx(6.0));
    // One (empty) keyframe list per segment keeps the two lists parallel.
    REQUIRE(vels.size() == 2);
}

TEST_CASE("loadPaths turns a two-point straight-line block into one cubic") {
    // A straight-line segment is two control points; degree-elevate it to the
    // equivalent cubic so every downstream segment is a valid 4-point cubic.
    ScratchFile file(
        "#PATH-START Path\n"
        "#POINTS-START\n"
        "0.0, 0.0\n"
        "3.0, 0.0\n"
        "#VELOCITIES-START\n");

    std::vector<std::vector<Point>> pts;
    std::vector<std::vector<KeyframeVelocitiesXandY>> vels;
    CHECK_NOTHROW(loadPaths(file.path, pts, vels));

    REQUIRE(pts.size() == 1);
    REQUIRE(pts[0].size() == 4);
    // Endpoints preserved, handles placed at the 1/3 and 2/3 marks: an exact
    // straight cubic.
    CHECK(pts[0][0].x == doctest::Approx(0.0));
    CHECK(pts[0][1].x == doctest::Approx(1.0));
    CHECK(pts[0][2].x == doctest::Approx(2.0));
    CHECK(pts[0][3].x == doctest::Approx(3.0));
    for (const auto& p : pts[0]) {
        CHECK(p.y == doctest::Approx(0.0));
    }
    REQUIRE(vels.size() == 1);
}

TEST_CASE("loadPaths rejects a points block that is neither a line nor a spline") {
    // Five points is not 2 (a line) and not 3N+1 (a spline), so it cannot be
    // split into whole cubics.
    ScratchFile file(
        "#PATH-START Path\n"
        "#POINTS-START\n"
        "0.0, 0.0\n"
        "1.0, 0.0\n"
        "2.0, 0.0\n"
        "3.0, 0.0\n"
        "4.0, 0.0\n"
        "#VELOCITIES-START\n");
    CHECK_THROWS_AS(load(file.path), FileParseError);
}

TEST_CASE("loadPaths routes each keyframe to the segment it lies on") {
    // Two cubics along +x from 0 to 6. A keyframe at x=1.5 sits on the first
    // segment, one at x=4.5 on the second; each must land in its own list.
    ScratchFile file(
        "#PATH-START Path\n"
        "#POINTS-START\n"
        "0.0, 0.0\n"
        "1.0, 0.0\n"
        "2.0, 0.0\n"
        "3.0, 0.0\n"
        "4.0, 0.0\n"
        "5.0, 0.0\n"
        "6.0, 0.0\n"
        "#VELOCITIES-START\n"
        "1.5, 0.0, 0.4\n"
        "4.5, 0.0, 0.6\n");

    std::vector<std::vector<Point>> pts;
    std::vector<std::vector<KeyframeVelocitiesXandY>> vels;
    CHECK_NOTHROW(loadPaths(file.path, pts, vels));

    REQUIRE(pts.size() == 2);
    REQUIRE(vels.size() == 2);
    REQUIRE(vels[0].size() == 1);
    REQUIRE(vels[1].size() == 1);
    CHECK(vels[0][0].x == doctest::Approx(1.5));
    CHECK(vels[1][0].x == doctest::Approx(4.5));
}
