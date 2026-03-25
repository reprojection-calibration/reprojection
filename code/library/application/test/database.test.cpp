#include "application/database.hpp"

#include <gtest/gtest.h>

#include "testing_utilities/temporary_file.hpp"

using namespace reprojection;

TEST(ApplicationDatabase, TestOpeningInputs) {
    auto result{application::Open("", "")};
    ASSERT_TRUE(std::holds_alternative<application::DbErrorMsg>(result));
    EXPECT_EQ(std::get<application::DbErrorMsg>(result).msg,
              "Provided workspace path: '' is not a valid directory - error code (2) with description 'No such file or "
              "directory'");

    std::filesystem::path const workspace{std::filesystem::temp_directory_path()};

    result = application::Open(workspace, "");
    ASSERT_TRUE(std::holds_alternative<application::DbErrorMsg>(result));
    EXPECT_EQ(std::get<application::DbErrorMsg>(result).msg,
              "Provided data source path: '' is not a valid file - error code (2) with description 'No such file or "
              "directory'");
}

// TODO THIS ALREADY EXISTS IN THE TESTING UTILITIES TESTING
std::size_t NumFilesInDir(std::filesystem::path const& path) {
    using std::filesystem::directory_iterator;
    return std::distance(directory_iterator(path), directory_iterator{});
}

TEST(ApplicationDatabase, TestDbCreation) {
    std::filesystem::path const workspace{std::filesystem::temp_directory_path()};
    testing_utilities::TemporaryFile const data_source(".bag", "serialized test data");

    // A corresponding database does not already exist so it will create one
    size_t files_before{NumFilesInDir(workspace)};
    auto result{application::Open(workspace, data_source.Path())};
    ASSERT_TRUE(std::holds_alternative<application::DbPtr>(result));
    EXPECT_TRUE(std::get<application::DbPtr>(result));  // Not null! Just a basic check.
    size_t files_after{NumFilesInDir(workspace)};
    EXPECT_EQ(files_after - files_before, 1);

    files_before = NumFilesInDir(workspace);
    result = application::Open(workspace, data_source.Path());
    ASSERT_TRUE(std::holds_alternative<application::DbPtr>(result));
    EXPECT_TRUE(std::get<application::DbPtr>(result));  // Not null! Just a basic check.
    files_after = NumFilesInDir(workspace);
    EXPECT_EQ(files_after - files_before, 0);
}