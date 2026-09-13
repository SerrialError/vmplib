#pragma once

#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

// Thrown for a command line the CLI cannot act on: an unknown flag, a flag
// with no value, a missing required flag, or a value of the wrong kind.
class CliError : public std::runtime_error {
public:
    explicit CliError(const std::string& what) : std::runtime_error(what) {}
};

// The `--flag value` pairs of one command line, keyed by flag name.
using FlagMap = std::map<std::string, std::string>;

// Parses args as `--flag value` pairs. Throws CliError for a flag not in
// allowed, a flag with no value after it, or a flag given twice.
FlagMap parseFlags(const std::vector<std::string>& args, const std::vector<std::string>& allowed);

// Throws CliError naming every flag in required that flags lacks, so the user
// can fix them all in one go instead of rerunning once per flag.
void requireFlags(const FlagMap& flags, const std::vector<std::string>& required);

// The value of flag, or fallback if it was not given.
std::string stringFlag(const FlagMap& flags, const std::string& flag, const std::string& fallback);

// The value of flag read as a number, or nullopt if it was not given. Throws
// CliError unless the whole value is a number, so "1.5m" is not read as 1.5.
std::optional<double> numberFlag(const FlagMap& flags, const std::string& flag);
