#include "database/sqlite_exception.hpp"

#include <gtest/gtest.h>

#include <sqlite3.h>

using namespace reprojection;

// TODO(Jack): This test uses sqlite3* and sqlite3_stmt* in a non-RAII safe way! Do not be inspired by this...
TEST(DatabaseSqliteException, TestSqliteException) {
    sqlite3* db{nullptr};
    sqlite3_open_v2(":memory:", &db, SQLITE_OPEN_READWRITE, nullptr);

    std::string const dummy_sql{"SELECT 1;"};
    sqlite3_stmt* stmt{nullptr};
    sqlite3_prepare_v2(db, dummy_sql.c_str(), -1, &stmt, nullptr);

    auto result{database::SqliteException(db, dummy_sql)};
    EXPECT_EQ(std::string(result.what()),
              "\n[SQLite Exception]\n----------------------------------------\nSQL Query:  SELECT 1;\n\nError Code : "
              "0\nError Msg  : not an error\n----------------------------------------");

    result = database::SqliteException(db, stmt);
    EXPECT_EQ(std::string(result.what()),
              "\n[SQLite Exception]\n----------------------------------------\nSQL Query:  SELECT 1;\n\nError Code : "
              "0\nError Msg  : not an error\n----------------------------------------");

    result = database::SqliteException(db);
    EXPECT_EQ(std::string(result.what()),
              "\n[SQLite Exception]\n----------------------------------------\n\nError Code : 0\nError Msg  : not an "
              "error\n----------------------------------------");

    result = database::SqliteException(stmt);
    EXPECT_EQ(std::string(result.what()),
              "\n[SQLite Exception]\n----------------------------------------\nSQL Query:  SELECT "
              "1;\n----------------------------------------");

    sqlite3_close_v2(db);
    sqlite3_finalize(stmt);
}
