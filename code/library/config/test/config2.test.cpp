#include "config/config2.hpp"

#include <gtest/gtest.h>

#include "testing_utilities/temporary_file.hpp"

using namespace reprojection;
using TemporaryFile = testing_utilities::TemporaryFile;

TEST(ConfigConfig2, TestXxx) {
    static constexpr std::string_view table_content{R"(
        [table]
        key1 = "value1"
    )"};
    TemporaryFile const config_file{".toml", table_content};

    EXPECT_EQ(1, 2);
}
