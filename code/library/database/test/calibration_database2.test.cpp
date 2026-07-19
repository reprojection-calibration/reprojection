#include "database/calibration_database2.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <string>

#include "testing_utilities/temporary_file.hpp"

using namespace reprojection;
using TemporaryFile = testing_utilities::TemporaryFile;

TEST(Yyy, Xxx) {
    TemporaryFile const temp_file{".db3"};

    EXPECT_FALSE(true);
}
