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

    testing_utilities::TemporaryFile const data_source(".test_extension", "serialized test data");

    result = application::Open(workspace, data_source.Path());
    EXPECT_TRUE(std::holds_alternative<application::DbPtr>(result));
}