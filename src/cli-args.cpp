#include "cli-args.hpp"

#include <algorithm>

FlagMap parseFlags(const std::vector<std::string>& args, const std::vector<std::string>& allowed) {
    FlagMap flags;
    for (size_t i = 0; i < args.size(); ++i) {
        const std::string& flag = args[i];
        if (std::find(allowed.begin(), allowed.end(), flag) == allowed.end()) {
            throw CliError("unrecognised argument '" + flag + "'");
        }
        // A following "--flag" is the next flag, not this one's value.
        if (i + 1 >= args.size() || args[i + 1].rfind("--", 0) == 0) {
            throw CliError(flag + " expects a value");
        }
        if (!flags.emplace(flag, args[++i]).second) {
            throw CliError(flag + " was given more than once");
        }
    }
    return flags;
}

void requireFlags(const FlagMap& flags, const std::vector<std::string>& required) {
    std::string missing;
    for (const std::string& flag : required) {
        if (flags.count(flag) == 0) {
            missing += " " + flag;
        }
    }
    if (!missing.empty()) {
        throw CliError("missing required" + missing);
    }
}

std::string stringFlag(const FlagMap& flags, const std::string& flag, const std::string& fallback) {
    const auto it = flags.find(flag);
    return it == flags.end() ? fallback : it->second;
}

std::optional<double> numberFlag(const FlagMap& flags, const std::string& flag) {
    const auto it = flags.find(flag);
    if (it == flags.end()) {
        return std::nullopt;
    }
    const std::string& text = it->second;
    try {
        size_t used = 0;
        const double value = std::stod(text, &used);
        if (used == text.size()) {
            return value;
        }
    } catch (const std::exception&) {
    }
    throw CliError(flag + " expects a number, got '" + text + "'");
}
