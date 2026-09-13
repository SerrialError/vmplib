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
