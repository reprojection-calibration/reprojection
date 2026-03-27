#include "application/io.hpp"

#include <gtest/gtest.h>

#include <ranges>

#include "testing_utilities/temporary_file.hpp"

using namespace reprojection;
using TemporaryFile = testing_utilities::TemporaryFile;

TEST(ApplicationCliUtils, TestParseCommandLineInput) {
    auto result{application::ParseCommandLineInput(0, nullptr)};
    ASSERT_TRUE(std::holds_alternative<application::CliErrorMsg>(result));
    EXPECT_EQ(std::get<application::CliErrorMsg>(result).msg, "Missing --config flag");

    char const arg0[]{"program"};
    char const arg1[]{"--config"};
    char const arg2[]{"tmp/config.toml"};
    char const arg3[]{"--data"};
    char const arg4[]{"tmp/data.bag"};
    char const arg5[]{"--workspace"};
    char const arg6[]{"tmp/workspace/"};

    char const* const argv[]{arg0, arg1, arg2, arg3, arg4, arg5, arg6};
    int argc{5};

    result = application::ParseCommandLineInput(argc, argv);
    ASSERT_TRUE(std::holds_alternative<application::PathConfig>(result));

    auto paths{std::get<application::PathConfig>(result)};
    EXPECT_EQ(paths.config_path, "tmp/config.toml");
    EXPECT_EQ(paths.data_path, "tmp/data.bag");
    EXPECT_EQ(paths.workspace_dir, "tmp");

    argc = 7;
    result = application::ParseCommandLineInput(argc, argv);
    ASSERT_TRUE(std::holds_alternative<application::PathConfig>(result));

    paths = std::get<application::PathConfig>(result);
    EXPECT_EQ(paths.config_path, "tmp/config.toml");
    EXPECT_EQ(paths.data_path, "tmp/data.bag");
    EXPECT_EQ(paths.workspace_dir, "tmp/workspace/");
}

TEST(ApplicationCliUtils, TestGetCommandOption) {
    auto result{application::GetCommandOption(nullptr, nullptr, "")};
    EXPECT_FALSE(result.has_value());

    char const arg0[]{"program"};
    char const arg1[]{"--key1"};
    char const arg2[]{"value1"};
    char const arg3[]{"--key2"};
    char const arg4[]{"value2"};

    char const* const argv[]{arg0, arg1, arg2, arg3, arg4};
    int const argc{5};

    result = application::GetCommandOption(argv, argv + argc, "--key1");
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(*result, "value1");

    result = application::GetCommandOption(argv, argv + argc, "--key2");
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(*result, "value2");

    result = application::GetCommandOption(argv, argv + argc, "--nonexistent_key");
    EXPECT_FALSE(result.has_value());
}

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

TEST(ApplicationLoadAndValidateConfig, TestHappyPath) {
    // TODO(Jack): This is now copy and pasted in three places, should we make one common def in the testing utils?
    static constexpr std::string_view minimum_config{R"(
        [sensor]
        camera_name = "/cam0/image_raw"
        camera_model = "double_sphere"

        [target]
        pattern_size = [3,4]
        type = "circle_grid"
    )"};
    TemporaryFile const config_file{".toml", minimum_config};

    auto const result{application::LoadAndValidateConfig(config_file.Path())};
    ASSERT_TRUE(std::holds_alternative<toml::table>(result));
    toml::table const gt_result{toml::parse(minimum_config)};
    EXPECT_EQ(std::get<toml::table>(result), gt_result);
}

TEST(ApplicationLoadAndValidateConfig, TestBadLoad) {
    auto const result{application::LoadAndValidateConfig("nonexistent.toml")};
    ASSERT_TRUE(std::holds_alternative<TomlErrorMsg>(result));
    EXPECT_EQ(std::get<TomlErrorMsg>(result).error, TomlError::FailedLoad);
    EXPECT_EQ(std::get<TomlErrorMsg>(result).msg,
              "Error parsing file 'nonexistent.toml' - File could not be opened for reading on line (0)");
}

TEST(ApplicationLoadAndValidateConfig, TestBadValidate) {
    static constexpr std::string_view empty_config{R"(
    )"};
    TemporaryFile const config_file{".toml", empty_config};

    auto const result{application::LoadAndValidateConfig(config_file.Path())};
    ASSERT_TRUE(std::holds_alternative<TomlErrorMsg>(result));
    EXPECT_EQ(std::get<TomlErrorMsg>(result).error, TomlError::MissingKey);
    EXPECT_EQ(std::get<TomlErrorMsg>(result).msg, "Configuration does not contain required key: sensor of type: table");
}