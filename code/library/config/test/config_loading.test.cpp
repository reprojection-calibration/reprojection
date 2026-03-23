#include "config/config_loading.hpp"

#include <gtest/gtest.h>

#include "testing_utilities/temporary_file.hpp"

using namespace reprojection;
using TemporaryFile = testing_utilities::TemporaryFile;

TEST(ConfigConfigLoading, TestLoadConfigFile) {
    static constexpr std::string_view table_content{R"(
        [table]
        key1 = "value1"
    )"};
    TemporaryFile const config_file{".toml", table_content};

    auto result{config::LoadConfigFile(config_file.Path())};
    EXPECT_TRUE(std::holds_alternative<toml::table>(result));

    result = config::LoadConfigFile("bad.toml");
    ASSERT_TRUE(std::holds_alternative<TomlErrorMsg>(result));
    EXPECT_EQ(std::get<TomlErrorMsg>(result).error, TomlError::FailedLoad);
    EXPECT_EQ(std::get<TomlErrorMsg>(result).msg,
              "Error parsing file 'bad.toml' - File could not be opened for reading on line (0)");
}

// Missing closing bracket for table header - tests the core tomlplusplus parsing error handling
TEST(ConfigConfigLoading, TestLoadConfigFileBadToml) {
    static constexpr std::string_view table_content{R"(
        [table
        key1 = "value1"
    )"};
    TemporaryFile const config_file{".toml", table_content};

    auto result{config::LoadConfigFile(config_file.Path())};
    ASSERT_TRUE(std::holds_alternative<TomlErrorMsg>(result));

    std::string const gt_result{"Error parsing file '" + config_file.Path().string() +
                                "' - Error while parsing table header: expected ']', saw '\\n' on line (2)"};
    EXPECT_EQ(std::get<TomlErrorMsg>(result).error, TomlError::FailedLoad);
    EXPECT_EQ(std::get<TomlErrorMsg>(result).msg, gt_result);
}