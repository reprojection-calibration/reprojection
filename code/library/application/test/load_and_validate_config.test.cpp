#include "application/load_and_validate_config.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

TEST(ApplicationLoadAndValidateConfig, TestYYYY) {
    //std::variant<toml::table, config::TomlErrorMsg>

    auto const result{application::LoadAndValidateConfig("")};

    EXPECT_EQ(1,2);
}