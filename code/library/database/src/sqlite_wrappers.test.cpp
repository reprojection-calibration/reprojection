#include "sqlite_wrappers.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

TEST(TestSqliteWrappers, TestSqlStatementError) {
    database::SqlitePtr db{nullptr};

    EXPECT_THROW(database::SqlStatement(db, ""), std::runtime_error);
}

TEST(TestSqliteWrappers, TestSqlTransactionError) {
    database::SqlitePtr db{nullptr};

    EXPECT_THROW(database::SqlTransaction{db}, std::runtime_error);
}