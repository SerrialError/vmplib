#include "mechanism-file.hpp"
#include "file-parser.hpp"

#include <cmath>
#include <fstream>
#include <optional>
#include <sstream>

namespace {

std::string trim(const std::string& text) {
    const size_t first = text.find_first_not_of(" \t\r");
    if (first == std::string::npos) {
        return "";
    }
    const size_t last = text.find_last_not_of(" \t\r");
    return text.substr(first, last - first + 1);
}

// Reads the whole of text as a finite number, so "0.8m" or "inf" is rejected
// instead of read as 0.8 or passed on to the profiler.
std::optional<double> parseNumber(const std::string& text) {
    try {
        size_t used = 0;
        const double value = std::stod(text, &used);
        if (used == text.size() && std::isfinite(value)) {
            return value;
        }
    } catch (const std::exception&) {
    }
    return std::nullopt;
}

// Hands out a file's meaningful lines one at a time -- comments stripped,
// whitespace trimmed, blank lines skipped -- and tags errors with file:line.
class LineReader {
public:
    explicit LineReader(const std::string& filename) : filename_(filename), in_(filename) {
        if (!in_) {
            throw FileParseError("could not open " + filename);
        }
    }

    bool next(std::string& line) {
        std::string raw;
        while (std::getline(in_, raw)) {
            ++line_;
            line = trim(raw.substr(0, raw.find("//")));
            if (!line.empty()) {
                return true;
            }
        }
        return false;
    }

    int lineNumber() const { return line_; }

    FileParseError error(const std::string& message) const { return error(line_, message); }

    FileParseError error(int line, const std::string& message) const {
        return FileParseError(filename_ + ":" + std::to_string(line) + ": " + message);
    }

private:
    std::string filename_;
    std::ifstream in_;
    int line_ = 0;
};

// A marker line split into the marker and whatever follows it.
struct Marker {
    std::string name;
    std::string argument;
};

Marker splitMarker(const std::string& line) {
    const size_t gap = line.find_first_of(" \t");
    if (gap == std::string::npos) {
        return Marker{ line, "" };
    }
    return Marker{ line.substr(0, gap), trim(line.substr(gap)) };
}

// Stores a marker's number in slot, which each move may set only once.
void setOnce(const LineReader& reader, const Marker& marker, std::optional<double>& slot) {
    if (slot) {
        throw reader.error(marker.name + " is given twice in this move");
    }
    slot = parseNumber(marker.argument);
    if (!slot) {
        throw reader.error(marker.name + " expects a number, got '" + marker.argument + "'");
    }
}

} // namespace

MoveFile loadMoves(const std::string& filename) {
    LineReader reader(filename);
    MoveFile file;

    // The move being read, until the next #MOVE-START or the end of the file.
    struct PendingMove {
        int line;
        std::string name;
        std::optional<double> from;
        std::optional<double> to;
        std::optional<double> endVelocity;
        std::vector<MoveKeyframe> keyframes;
    };
    std::optional<PendingMove> pending;
    bool readingKeyframes = false;

    const auto finishMove = [&]() {
        if (!pending) {
            return;
        }
        const std::string label = pending->name.empty() ? "move" : "move '" + pending->name + "'";
        if (!pending->to) {
            throw reader.error(pending->line, label + " has no #TO");
        }
        if (pending->from) {
            if (file.moves.empty()) {
                file.startPosition = *pending->from;
            } else if (*pending->from != file.moves.back().target) {
                std::ostringstream msg;
                msg << label << " starts #FROM " << *pending->from
                    << " but the move before it ends at " << file.moves.back().target;
                throw reader.error(pending->line, msg.str());
            }
        }
        file.moves.push_back(ScalarMove{ *pending->to, pending->endVelocity.value_or(0.0),
                                         std::move(pending->keyframes) });
        pending.reset();
    };

    std::string line;
    while (reader.next(line)) {
        if (line[0] != '#') {
            if (!readingKeyframes) {
                throw reader.error("expected a #MARKER line, got '" + line + "'");
            }
            const size_t comma = line.find(',');
            std::optional<double> position;
            std::optional<double> speed;
            if (comma != std::string::npos) {
                position = parseNumber(trim(line.substr(0, comma)));
                speed = parseNumber(trim(line.substr(comma + 1)));
            }
            if (!position || !speed) {
                throw reader.error("expected a keyframe as 'position, speed', got '" + line + "'");
            }
            pending->keyframes.push_back(MoveKeyframe{ *position, *speed });
            continue;
        }

        const Marker marker = splitMarker(line);
        readingKeyframes = false;
        if (marker.name == "#MOVE-START") {
            finishMove();
            pending = PendingMove{ reader.lineNumber(), marker.argument, std::nullopt,
                                   std::nullopt, std::nullopt, {} };
            continue;
        }
        const bool known = marker.name == "#FROM" || marker.name == "#TO" ||
                           marker.name == "#END-VELOCITY" || marker.name == "#KEYFRAMES-START";
        if (!known) {
            throw reader.error("unknown marker " + marker.name);
        }
        if (!pending) {
            throw reader.error(marker.name + " must come after a #MOVE-START");
        }
        if (marker.name == "#FROM") {
            setOnce(reader, marker, pending->from);
        } else if (marker.name == "#TO") {
            setOnce(reader, marker, pending->to);
        } else if (marker.name == "#END-VELOCITY") {
            setOnce(reader, marker, pending->endVelocity);
        } else {
            if (!marker.argument.empty()) {
                throw reader.error("#KEYFRAMES-START takes no value");
            }
            readingKeyframes = true;
        }
    }
    finishMove();

    if (file.moves.empty()) {
        throw FileParseError(filename + ": no moves; each move begins with #MOVE-START");
    }
    return file;
}
