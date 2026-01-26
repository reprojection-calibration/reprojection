#include "config/load_configuration.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

using namespace reprojection;

// TODO THIS BELONGS IN A COMMON TESTING PACKAGE!
class TempFile {
   public:
    TempFile(std::string_view const& contents) {
        // ERROR NAME IS NOT RANDOM WE WILL GET CONFLICTS!
        path_ = std::filesystem::temp_directory_path() / "test_config_xxx.toml";

        std::ofstream out(path_);
        if (not out) {
            throw std::runtime_error("Failed to create temp file at path: " + path_.string());
        }
        out << contents;
    }

    ~TempFile() { std::filesystem::remove(path_); }

    std::filesystem::path Path() const { return path_; }

   private:
    std::filesystem::path path_;
};

TEST(ConfigLoadConfiguration, TestLoadConfiguration) {
    static constexpr std::string_view happy_path_config{R"(
        [solver]
        minimizer_type = "LINE_SEARCH"
        min_line_search_step_size = 1e-6
    )"};
    TempFile const config_file{happy_path_config};

    ceres::Solver::Options const config{config::LoadConfiguration(config_file.Path())};
    EXPECT_EQ(config.minimizer_type, ceres::LINE_SEARCH);
    EXPECT_EQ(config.min_line_search_step_size, 1e-6);
}

TEST(ConfigLoadConfiguration, TestLoadConfigurationEmptyConfig) {
    static constexpr std::string_view happy_path_config{R"(
        [some_other_config]
        blah = 1
    )"};
    TempFile const config_file{happy_path_config};

    // There is no [solver] config to load, so we will instead get a default initialized ceres::Solver::Options back :)
    ceres::Solver::Options const config{config::LoadConfiguration(config_file.Path())};
    EXPECT_EQ(config.minimizer_type, ceres::TRUST_REGION);
    EXPECT_EQ(config.min_line_search_step_size, 1e-9);
}