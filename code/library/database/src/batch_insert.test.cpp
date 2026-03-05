#include "batch_insert.hpp"

#include <gtest/gtest.h>

#include <string>

#include "test_fixture.hpp"

using namespace reprojection;

TEST_F(SqliteTestFixture, TestBatchInsert) {
    CreateExampleTable();

    EXPECT_NO_THROW(InsertRecordIds());
}