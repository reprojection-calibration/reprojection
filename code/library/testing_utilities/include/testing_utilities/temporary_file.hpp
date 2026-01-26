#pragma once

#include <filesystem>

namespace reprojection::testing_utilities {

class TemporaryFile {
   public:
    // NOTE(Jack): Technically the file extension does not physically matter. For example a file does not need to have a
    // .toml or .db3 extension in order for a toml or sqlite library to ingest those files. However, for human
    // readability and reasoning we include the option to pass the extension so it is clear what kind of file we are
    // creating in what context.
    explicit TemporaryFile(std::string const& extension);

    // TODO(Jack): If at some future date we need to create temporary with files with something else besides text, we
    //  could template this method to accept any method that has a valid << operator which works with streams. But for
    //  now we do not need that and therefore we are happy to hardcode it to only work with string views.
    explicit TemporaryFile(std::string const& extension, std::string_view const& contents);

    ~TemporaryFile();

    std::filesystem::path Path() const;

   private:
    std::filesystem::path path_;
};

}  // namespace reprojection::testing_utilities