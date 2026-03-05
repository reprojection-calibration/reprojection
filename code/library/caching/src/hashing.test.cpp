#include "hashing.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

TEST(CachingHashing, TestHash) {
    std::string const result{caching::Sha256("Jack")};
    EXPECT_EQ(result, "b5fd03dd91df1cfbd2f19c115d24d58bbda01a23fb01924bb78b2cc14f7ff1cb");
}