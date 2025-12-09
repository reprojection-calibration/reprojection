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
    // Returns false because we cannot create duplicated table (use CREATE TABLE IF NOT EXISTS to silently pass this)
    bool const table_duplicated{database::Sqlite3Tools::Execute(data_table_sql_, db)};
    EXPECT_FALSE(table_duplicated);

    bool const values_added{database::Sqlite3Tools::Execute(add_data_sql_, db)};
    EXPECT_TRUE(values_added);

    sqlite3_close(db);
}

TEST_F(TempFolderDummySql, TestExecuteCallback) {
    std::string const record{database_path_ + "/record_sss.db3"};

    sqlite3* db;
    sqlite3_open(record.c_str(), &db);

    // Create the table and fill it with some values but ignore return codes, they are tested above
    static_cast<void>(database::Sqlite3Tools::Execute(data_table_sql_, db));
    static_cast<void>(database::Sqlite3Tools::Execute(add_data_sql_, db));

    std::string const select_all_data_sql{"SELECT value FROM example_data;"};
    auto callback = [](void* data, int, char** argv, char**) -> int {
        auto* vec = reinterpret_cast<std::vector<double>*>(data);
        vec->push_back(std::stod(argv[0]));  // Hardcoded because only one value per row

        return 0;
    };
    std::vector<double> values;

    bool const data_selected{database::Sqlite3Tools::Execute(select_all_data_sql, db, callback, &values)};
    EXPECT_TRUE(data_selected);
    EXPECT_EQ(std::size(values), 3);
    EXPECT_FLOAT_EQ(values[0], 0);
    EXPECT_FLOAT_EQ(values[1], 1.1);
    EXPECT_FLOAT_EQ(values[2], 2.2);

    sqlite3_close(db);
}