#include "doctest.h"
#include "printer.hpp"

#include <sstream>
#include <vector>

namespace {

const std::vector<ScalarSample> kSamples = {
    {0.0, 0.0, 0.0, 0.0},
    {0.0015, 0.3, 30.0, 0.01},
};

} // namespace

TEST_CASE("1D samples print as three Desmos lists against time") {
    std::ostringstream out;
    Printer::printScalarSamplesDesmos(out, kSamples);
    CHECK(out.str() == "P = [(0.000000,0.000000),(0.010000,0.001500)]\n"
                       "V = [(0.000000,0.000000),(0.010000,0.300000)]\n"
                       "A = [(0.000000,0.000000),(0.010000,30.000000)]\n");
}

TEST_CASE("1D samples print as a C++ initialiser list") {
    std::ostringstream out;
    Printer::printScalarSamplesCode(out, kSamples);
    CHECK(out.str() ==
          "S = {{0.000000, 0.000000, 0.000000},{0.001500, 0.300000, 30.000000}};\n");
}

TEST_CASE("velocity samples print as two Desmos lists against time") {
    const std::vector<VelocitySample> samples = {{0.0, 0.0, 0.0}, {8.0, 800.0, 0.01}};
    std::ostringstream out;
    Printer::printVelocitySamplesDesmos(out, samples);
    CHECK(out.str() == "V = [(0.000000,0.000000),(0.010000,8.000000)]\n"
                       "A = [(0.000000,0.000000),(0.010000,800.000000)]\n");
}

TEST_CASE("velocity samples print as a C++ initialiser list") {
    const std::vector<VelocitySample> samples = {{0.0, 0.0, 0.0}, {8.0, 800.0, 0.01}};
    std::ostringstream out;
    Printer::printVelocitySamplesCode(out, samples);
    CHECK(out.str() == "S = {{0.000000, 0.000000},{8.000000, 800.000000}};\n");
}
