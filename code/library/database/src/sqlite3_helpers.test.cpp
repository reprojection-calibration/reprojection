#include "sqlite3_helpers.hpp"

#include <gtest/gtest.h>

#include <sqlite3.h>

#include <filesystem>
#include <string>

using namespace reprojection;

TEST(DatabaseSqlite3Helpers, TestBindErrors) {
    EXPECT_THROW(database::Sqlite3Tools::Bind(nullptr, 1, ""), std::runtime_error);
    EXPECT_THROW(database::Sqlite3Tools::Bind(nullptr, 1, static_cast<int64_t>(123)), std::runtime_error);
    EXPECT_THROW(database::Sqlite3Tools::Bind(nullptr, 1, 1.23), std::runtime_error);
    EXPECT_THROW(database::Sqlite3Tools::BindBlob(nullptr, 1, {}), std::runtime_error);
}

TEST(DatabaseSqlite3Helpers, TestStepRowError) {
    EXPECT_THROW(database::Sqlite3Tools::StepRow(nullptr), std::runtime_error);
}

TEST(DatabaseSqlite3Helpers, TestSqliteBlobError) {
    auto const result{database::Sqlite3Tools::SqliteBlob(nullptr, -1)};
    EXPECT_TRUE(std::empty(result));
}