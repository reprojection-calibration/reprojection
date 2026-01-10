#include "sqlite3_helpers.hpp"

#include <gtest/gtest.h>
#include <sqlite3.h>

#include <filesystem>
#include <string>

using namespace reprojection;

// WARN PARTIALLY COPY AND PASTED from TempFolder!!!
class TempFolderDummySql : public ::testing::Test {
   protected:
    // cppcheck-suppress unusedFunction
    void SetUp() override { std::filesystem::create_directories(database_path_); }

    // cppcheck-suppress unusedFunction
    void TearDown() override { std::filesystem::remove_all(database_path_); }

    std::string database_path_{"sandbox"};

    // cppcheck-suppress unusedStructMember
    std::string const data_table_sql_{
        "CREATE TABLE example_data ("
        "record_id INTEGER, "
        "value REAL NOT NULL, "
        "PRIMARY KEY (record_Id)"
        ");"};

    // cppcheck-suppress unusedStructMember
    std::string const add_data_sql_{
        "INSERT INTO example_data (value) "
        "VALUES (0.0), (1.1), (2.2);"};
};

// Test where we create a simple auto incremented table and add some values
TEST_F(TempFolderDummySql, TestExecute) {
    std::string const record{database_path_ + "/record_lll.db3"};

    sqlite3* db;
    sqlite3_open(record.c_str(), &db);

    bool const table_created{database::Sqlite3Tools::Execute(data_table_sql_, db)};
    ASSERT_TRUE(table_created);

    // TODO(Jack): Capture and test stderr output without using gtest internal API!
    // Returns false because we cannot create duplicated table. Use CREATE TABLE IF NOT EXISTS if you want to silently
    // handle this.
    bool const table_duplicated{database::Sqlite3Tools::Execute(data_table_sql_, db)};
    EXPECT_FALSE(table_duplicated);

    bool const values_added{database::Sqlite3Tools::Execute(add_data_sql_, db)};
    EXPECT_TRUE(values_added);

    sqlite3_close(db);
}

TEST(TestSqlite3Helpers, TestBindFailures) {
    EXPECT_THROW(database::Sqlite3Tools::Bind(nullptr, 1, ""), std::runtime_error);
    EXPECT_THROW(database::Sqlite3Tools::Bind(nullptr, 1, static_cast<int64_t>(123)), std::runtime_error);
    EXPECT_THROW(database::Sqlite3Tools::Bind(nullptr, 1, 1.23), std::runtime_error);
    EXPECT_THROW(database::Sqlite3Tools::BindBlob(nullptr, 1, nullptr, -1), std::runtime_error);
}