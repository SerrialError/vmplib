#pragma once

#include <cstdio>
#include <fstream>
#include <string>
#include <unistd.h>

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
