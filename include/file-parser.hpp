// file-parser.hpp
#ifndef PATH_LOADER_HPP
#define PATH_LOADER_HPP

#include "types.hpp"
#include <stdexcept>
#include <string>
#include <vector>

// Thrown when the input file cannot be opened or contains a line the parser
// cannot make sense of. Reporting these to cerr and returning normally left the
// caller with a partial, silently-wrong trajectory and a success exit status.
class FileParseError : public std::runtime_error {
public:
    explicit FileParseError(const std::string& what) : std::runtime_error(what) {}
};

// Parses the given file and fills controlPoints and keyFrameVelocityList.
// Each time the marker is encountered, a new path block begins.
// Throws FileParseError if the file cannot be opened or a line is malformed.
void loadPaths(
    const std::string& filename,
    std::vector<std::vector<Point>>& controlPoints,
    std::vector<std::vector<KeyframeVelocitiesXandY>>& keyFrameVelocityList
);

#endif // FILE_PARSER_HPP
