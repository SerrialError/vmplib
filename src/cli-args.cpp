#include "cli-args.hpp"

#include <algorithm>
#include <cctype>

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

namespace {
// A Rust identifier: a letter or underscore, then letters, digits or
// underscores. Raw identifiers, r#type, are not worth supporting for a path
// naming a struct.
bool isIdentifier(const std::string& text) {
    const auto isWordChar = [](unsigned char c) { return std::isalnum(c) != 0 || c == '_'; };
    if (text.empty() || (std::isdigit(static_cast<unsigned char>(text.front())) != 0)) {
        return false;
    }
    return std::all_of(text.begin(), text.end(), isWordChar);
}
} // namespace

std::string rustPathFlag(const FlagMap& flags, const std::string& flag) {
    const auto it = flags.find(flag);
    if (it == flags.end()) {
        return {};
    }
    const std::string& text = it->second;

    // A leading :: is the absolute form, ::my_crate::Type.
    size_t at = text.rfind("::", 0) == 0 ? 2 : 0;
    bool valid = at < text.size();
    while (valid) {
        const size_t separator = text.find("::", at);
        const size_t end = separator == std::string::npos ? text.size() : separator;
        valid = isIdentifier(text.substr(at, end - at));
        if (separator == std::string::npos) {
            break;
        }
        at = separator + 2;
    }
    if (!valid) {
        throw CliError(flag + " expects a Rust path like crate::module::Type, got '" + text + "'");
    }
    return text;
}
