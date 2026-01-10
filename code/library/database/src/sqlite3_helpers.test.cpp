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
};

// Test where we create a simple auto incremented table and add some values
TEST_F(TempFolderDummySql, TestExecute) {
    std::string const record{database_path_ + "/record_lll.db3"};
    sqlite3* db;
    sqlite3_open(record.c_str(), &db);

    std::string const data_table_sql_{
        "CREATE TABLE example_data_table ("
        "record_id INTEGER, "
        "value REAL NOT NULL, "
        "PRIMARY KEY (record_Id)"
        ");"};
    bool const table_created{database::Sqlite3Tools::Execute(data_table_sql_, db)};
    ASSERT_TRUE(table_created);

    // TODO(Jack): Capture and test stderr output without using gtest internal API!
    // Returns false because we cannot create duplicated table. Use CREATE TABLE IF NOT EXISTS if you want to silently
    // handle this.
    bool const duplicated_table_created{database::Sqlite3Tools::Execute(data_table_sql_, db)};
    EXPECT_FALSE(duplicated_table_created);

    std::string const add_data_sql_{
        "INSERT INTO example_data_table (value) "
        "VALUES (0.0), (1.1), (2.2);"};
    bool const values_added{database::Sqlite3Tools::Execute(add_data_sql_, db)};
    EXPECT_TRUE(values_added);

    sqlite3_close(db);
}

TEST_F(TempFolderDummySql, TestAddBlob) {
    std::string const record{database_path_ + "/record_lll.db3"};
    sqlite3* db;
    sqlite3_open(record.c_str(), &db);

    std::string const blob_table_sql_{
        "CREATE TABLE example_blob_table ("
        "timestamp_ns INTEGER NOT NULL, "
        "sensor_name TEXT NOT NULL, "
        "data BLOB NOT NULL "
        ");"};
    bool const table_created{database::Sqlite3Tools::Execute(blob_table_sql_, db)};
    ASSERT_TRUE(table_created);

    std::string const add_blob_sql_{
        "INSERT INTO example_blob_table (timestamp_ns, sensor_name, data) "
        "VALUES (?, ?, ?);"};

    // Success
    std::string const buffer{"the future is calibrated"};
    auto result{
        database::Sqlite3Tools::AddBlob(add_blob_sql_, 0, "/cam/retro/123", buffer.c_str(), std::size(buffer), db)};
    ASSERT_TRUE(std::holds_alternative<database::SqliteFlag>(result));
    EXPECT_EQ(std::get<database::SqliteFlag>(result), database::SqliteFlag::Ok);

    // Failure case "failed step" - blob data itself is bad (i.e. nullptr)
    result = database::Sqlite3Tools::AddBlob(add_blob_sql_, 0, "/cam/retro/123", nullptr, -1, db);
    ASSERT_TRUE(std::holds_alternative<database::SqliteErrorCode>(result));
    EXPECT_EQ(std::get<database::SqliteErrorCode>(result), database::SqliteErrorCode::FailedStep);

    // Failure case "failed binding" - malformed sql statement does not match the table in the database (its missing
    // 'data'!)
    std::string const malformed_add_blob_sql_{
        "INSERT INTO example_blob_table (timestamp_ns, sensor_name) "
        "VALUES (?, ?);"};
    result = database::Sqlite3Tools::AddBlob(malformed_add_blob_sql_, 0, "/cam/retro/123", buffer.c_str(),
                                             std::size(buffer), db);
    ASSERT_TRUE(std::holds_alternative<database::SqliteErrorCode>(result));
    EXPECT_EQ(std::get<database::SqliteErrorCode>(result), database::SqliteErrorCode::FailedBinding);
}

TEST(TestSqlite3Helpers, TestBindErrors) {
    EXPECT_THROW(database::Sqlite3Tools::Bind(nullptr, 1, ""), std::runtime_error);
    EXPECT_THROW(database::Sqlite3Tools::Bind(nullptr, 1, static_cast<int64_t>(123)), std::runtime_error);
    EXPECT_THROW(database::Sqlite3Tools::Bind(nullptr, 1, 1.23), std::runtime_error);
    EXPECT_THROW(database::Sqlite3Tools::BindBlob(nullptr, 1, nullptr, -1), std::runtime_error);
}