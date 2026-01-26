#include "testing_utilities/temporary_file.hpp"

#include <cstdio>
#include <fstream>

namespace reprojection::testing_utilities {

TemporaryFile::TemporaryFile(std::string const& extension) {
    std::string const random_name{std::tmpnam(nullptr)};
    path_ = std::filesystem::temp_directory_path() / (random_name + extension);
}

TemporaryFile::TemporaryFile(std::string const& extension, std::string_view const& contents) : TemporaryFile(extension) {
    std::ofstream out(path_);
    if (not out) {
        throw std::runtime_error("Failed to create temp file at path: " + path_.string());
    }
    out << contents;
}

TemporaryFile::~TemporaryFile() { std::filesystem::remove(path_); }

std::filesystem::path TemporaryFile::Path() const { return path_; }

}  // namespace reprojection::testing_utilities