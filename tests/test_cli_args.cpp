#include "doctest.h"
#include "cli-args.hpp"

#include <string>
#include <vector>

namespace {

const std::vector<std::string> kAllowed = {"--file", "--max-vel", "--out"};

} // namespace

TEST_CASE("parseFlags reads --flag value pairs") {
    const FlagMap flags = parseFlags({"--file", "a.txt", "--max-vel", "1.5"}, kAllowed);
    CHECK(flags.size() == 2);
    CHECK(flags.at("--file") == "a.txt");
    CHECK(flags.at("--max-vel") == "1.5");
}

TEST_CASE("parseFlags accepts a negative number as a value") {
    const FlagMap flags = parseFlags({"--max-vel", "-1.5"}, kAllowed);
    CHECK(flags.at("--max-vel") == "-1.5");
}

TEST_CASE("parseFlags rejects unknown, valueless and repeated flags") {
    CHECK_THROWS_WITH_AS(parseFlags({"--bogus", "1"}, kAllowed),
                         "unrecognised argument '--bogus'", CliError);
    CHECK_THROWS_WITH_AS(parseFlags({"--file"}, kAllowed), "--file expects a value", CliError);
    // The next flag is not a value for this one.
    CHECK_THROWS_WITH_AS(parseFlags({"--file", "--out", "o.txt"}, kAllowed),
                         "--file expects a value", CliError);
    CHECK_THROWS_WITH_AS(parseFlags({"--file", "a", "--file", "b"}, kAllowed),
                         "--file was given more than once", CliError);
}

TEST_CASE("requireFlags names every missing flag at once") {
    const FlagMap flags = parseFlags({"--out", "o.txt"}, kAllowed);
    CHECK_THROWS_WITH_AS(requireFlags(flags, {"--file", "--max-vel", "--out"}),
                         "missing required --file --max-vel", CliError);
    CHECK_NOTHROW(requireFlags(flags, {"--out"}));
}

TEST_CASE("stringFlag falls back when the flag is absent") {
    const FlagMap flags = parseFlags({"--out", "o.txt"}, kAllowed);
    CHECK(stringFlag(flags, "--out", "output.txt") == "o.txt");
    CHECK(stringFlag(flags, "--file", "none") == "none");
}

TEST_CASE("numberFlag reads only a value that is entirely a number") {
    CHECK(*numberFlag(parseFlags({"--max-vel", "1.5"}, kAllowed), "--max-vel") ==
          doctest::Approx(1.5));
    CHECK_FALSE(numberFlag(parseFlags({}, kAllowed), "--max-vel").has_value());
    for (const char* bad : {"1.5m", "fast", " ", "1.5 2"}) {
        CAPTURE(bad);
        CHECK_THROWS_AS(numberFlag(parseFlags({"--max-vel", bad}, kAllowed), "--max-vel"),
                        CliError);
    }
}

TEST_CASE("rustPathFlag takes identifiers separated by ::") {
    const std::vector<std::string> allowed = {"--rust-type"};
    CHECK(rustPathFlag(parseFlags({}, allowed), "--rust-type").empty());

    for (const char* good : {"DriveSample", "crate::motion_profile::DriveSample",
                             "::my_crate::Sample", "super::_step9"}) {
        CAPTURE(good);
        CHECK(rustPathFlag(parseFlags({"--rust-type", good}, allowed), "--rust-type") == good);
    }

    // Anything else would land in the generated file as it stands.
    for (const char* bad : {"crate::", "::", "9lives", "crate::9", "two words", "a:b",
                            "drop; mod evil", "crate::Type "}) {
        CAPTURE(bad);
        CHECK_THROWS_AS(rustPathFlag(parseFlags({"--rust-type", bad}, allowed), "--rust-type"),
                        CliError);
    }
}
