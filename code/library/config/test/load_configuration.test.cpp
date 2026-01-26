#include "config/load_configuration.hpp"

#include <gtest/gtest.h>

#include <cstdio>  // FOR TEMPNAME
#include <filesystem>
#include <fstream>

using namespace reprojection;

// TODO THIS BELONGS IN A COMMON TESTING PACKAGE! CAN WE ALSO USE THIS FOR THE DB TESTING?
class TempFile {
   public:
    TempFile() {
        std::string const random_name{std::tmpnam(nullptr)};
        path_ = std::filesystem::temp_directory_path() / (random_name + ".toml");
    }

    explicit TempFile(std::string_view const& contents) : TempFile() {
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

// There is no [solver] config to load, so we will instead get a default initialized ceres::Solver::Options back :)
TEST(ConfigLoadConfiguration, TestLoadConfigurationDefaultSolverOptions) {
    static constexpr std::string_view no_solver_parameters_config{R"(
        [some_other_config]
        blah = 1
    )"};
    TempFile const config_file{no_solver_parameters_config};

    ceres::Solver::Options const config{config::LoadConfiguration(config_file.Path())};
    EXPECT_EQ(config.minimizer_type, ceres::TRUST_REGION);
    EXPECT_EQ(config.min_line_search_step_size, 1e-9);
}

TEST(XXX, XXX) { TempFile const config_file; }