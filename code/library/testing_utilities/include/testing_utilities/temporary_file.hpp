#pragma once

#include <filesystem>

namespace reprojection::testing_utilities {

class TemporaryFile {
   public:
    explicit TemporaryFile(std::string const& extension);

    explicit TemporaryFile(std::string const& extension, std::string_view const& contents);

    ~TemporaryFile();

    std::filesystem::path Path() const;

   private:
    std::filesystem::path path_;
};

}  // namespace reprojection::testing_utilities