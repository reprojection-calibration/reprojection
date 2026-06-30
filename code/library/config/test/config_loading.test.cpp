#include "config/config_loading.hpp"

#include <gtest/gtest.h>

#include <format>

#include "testing_utilities/temporary_file.hpp"

using namespace reprojection;
using TemporaryFile = testing_utilities::TemporaryFile;

TEST(ConfigConfigLoading, TestLoadConfigFile) {
    static constexpr std::string_view content{R"(
        [table]
    )"};
    TemporaryFile const config_file{".toml", content};

    EXPECT_NO_THROW(config::LoadConfigFile(config_file.Path()));
}

TEST(ConfigConfigLoading, TestLoadConfigFileBadToml) {
    static constexpr std::string_view misformatted{R"(
        [table
    )"};
    TemporaryFile const misformatted_config_file{".toml", misformatted};

    // NOTE(Jack): Gtest does not provide us a direct way to check both that a function throws and what the value of the
    // exception actually is. Therefore, we engineer that here ourselves but putting in a try catch block with a
    // assertion inside the EXPECT_THROW assertion. A little hacky but it gets the job done.
    EXPECT_THROW(
        try { config::LoadConfigFile(misformatted_config_file.Path()); } catch (std::runtime_error const& e) {
            // NOTE(Jack): We cannot hardcode the path file in the expected error message because that is random and
            // changes every time.
            std::string const error_msg{std::format(
                "{{'file': '{}', 'line': 2, 'error': 'Error while parsing table header: expected ']', saw '\\n''}}",
                misformatted_config_file.Path().string())};
            EXPECT_EQ(std::string(e.what()), error_msg);

            // Rethrow so we satisfy the EXPECT_THROW assertion.
            throw;
        },
        std::runtime_error);
}