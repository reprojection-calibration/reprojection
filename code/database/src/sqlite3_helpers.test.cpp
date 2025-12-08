#include "sqlite3_helpers.hpp"

#include <gtest/gtest.h>
#include <sqlite3.h>

#include <filesystem>
#include <string>

using namespace reprojection;

// WARN COPY AND PASTED
class TempFolder : public ::testing::Test {
   protected:
    void SetUp() override { std::filesystem::create_directories(database_path_); }

    void TearDown() override { std::filesystem::remove_all(database_path_); }

    std::string database_path_{"sandbox"};
};

std::string const init_dummy_data_table_sql{
    "CREATE TABLE example_data ("
    "record_id INTEGER, "  // Alias for the ROWID which is included and incremented by sqlite for each record row
    "value REAL NOT NULL, "
    "PRIMARY KEY (record_Id)"
    ");"};

std::string const add_value_sql{
    "INSERT INTO example_data (value) "
    "VALUES (0.0), (1.1), (2.2);"};

// Test where we create a simple auto incremented table and add some values using
TEST_F(TempFolder, TestExecute) {
    std::string const record{database_path_ + "/record_lll.db3"};

    sqlite3* db;
    sqlite3_open(record.c_str(), &db);

    bool const table_created{database::Sqlite3Tools::Execute(init_dummy_data_table_sql, db)};
    ASSERT_TRUE(table_created);

    // Returns false because we cannot create duplicated table (use CREATE TABLE IF NOT EXISTS to silently pass this)
    bool const table_duplicated{database::Sqlite3Tools::Execute(init_dummy_data_table_sql, db)};
    EXPECT_FALSE(table_duplicated);

    bool const values_added{database::Sqlite3Tools::Execute(add_value_sql, db)};
    EXPECT_TRUE(values_added);

    sqlite3_close(db);
}

TEST_F(TempFolder, TestExecuteCallback) {
    std::string const record{database_path_ + "/record_sss.db3"};

    sqlite3* db;
    sqlite3_open(record.c_str(), &db);

    // Create the table and fill it with some values
    static_cast<void>(database::Sqlite3Tools::Execute(init_dummy_data_table_sql, db));
    static_cast<void>(database::Sqlite3Tools::Execute(add_value_sql, db));

    std::string const select_all_data_sql{"SELECT value FROM example_data;"};
    std::vector<double> values;

    auto callback = [](void* data, int argc, char** argv, char** col_name) -> int {
        static_cast<void>(argc);
        static_cast<void>(col_name);

        auto* vec = reinterpret_cast<std::vector<double>*>(data);
        vec->push_back(std::stod(argv[0]));  // Hardcoded: one value per row

        return 0;
    };

    bool const data_selected{database::Sqlite3Tools::Execute(select_all_data_sql, db, callback, &values)};
    EXPECT_TRUE(data_selected);
    EXPECT_EQ(std::size(values), 3);
    EXPECT_FLOAT_EQ(values[0], 0);
    EXPECT_FLOAT_EQ(values[1], 1.1);
    EXPECT_FLOAT_EQ(values[2], 2.2);

    sqlite3_close(db);
}