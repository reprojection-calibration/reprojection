#include "testing_utilities/temporary_file.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <string_view>

using namespace reprojection;

std::size_t TestFileCount(std::filesystem::path const& path) {
    auto is_test_file = [](std::filesystem::path const& path) { return path.extension() == ".test_extension"; };

    return std::count_if(std::filesystem::directory_iterator(path), std::filesystem::directory_iterator{},
                         is_test_file);
}

// Simply passing in a file extension will not result in a file itself being created.
TEST(TestingUtilitiesTemporaryFile, TestPathOnly) {
    // NOTE(Jack): The temp_directory_path() function here is also used inside the TemporaryFile class.
    std::filesystem::path const tmp_folder{std::filesystem::temp_directory_path()};

    EXPECT_EQ(TestFileCount(tmp_folder), 0);
    testing_utilities::TemporaryFile const temp_file{".test_extension"};
    EXPECT_EQ(TestFileCount(tmp_folder), 0);
}

TEST(TestingUtilitiesTemporaryFile, TestUniqueNames) {
    testing_utilities::TemporaryFile const temp_file{".test_extension"};
    testing_utilities::TemporaryFile const temp_file_1{".test_extension"};

    // The file names will be different but the extensions will be the same. Of course, we cannot test exhaustively that
    // every temporary file will be unique, so we just check that for two files they are unique.
    EXPECT_NE(temp_file.Path(), temp_file_1.Path());
    EXPECT_EQ(temp_file.Path().extension(), temp_file_1.Path().extension());
}

// We create a couple files with text content in a local namespace, check that they exist, and then check afterwards
// once the namespace is out of scope and the temporary file destructors called, that there the files are gone.
TEST(TestingUtilitiesTemporaryFile, TestFileCreation) {
    std::filesystem::path const tmp_folder{std::filesystem::temp_directory_path()};
    EXPECT_EQ(TestFileCount(tmp_folder), 0);

    {
        static constexpr std::string_view example_text{R"(some random piece of text in a string view)"};
        testing_utilities::TemporaryFile const temp_file{".test_extension", example_text};
        testing_utilities::TemporaryFile const temp_file_1{".test_extension", example_text};
        testing_utilities::TemporaryFile const temp_file_2{".test_extension", example_text};

        EXPECT_EQ(TestFileCount(tmp_folder), 3);
    }

    EXPECT_EQ(TestFileCount(tmp_folder), 0);
}