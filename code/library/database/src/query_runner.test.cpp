#include "query_runner.hpp"

#include <gtest/gtest.h>

#include <string>

#include "batch_insert.hpp"
#include "sqlite3_helpers.hpp"
#include "statement_executor.hpp"

using namespace reprojection;

// TODO(Jack): Use test fixture!

TEST(DatabaseBatchInsert, TestExecuteQuery) {
    sqlite3* db;
    sqlite3_open(":memory:", &db);

    // Create table
    std::string const data_table_sql{
        "CREATE TABLE example_data_table ("
        "record_id INTEGER "
        ");"};
    database::ExecuteStatement(data_table_sql, db);

    // Insert a batch into it
    std::string const insert_sql{
        "INSERT INTO example_data_table (record_id)"
        "VALUES (?);"};
    std::vector<int64_t> const example_record_ids{0, 1, 2, 3, 4, 5};

    database::BatchInsert(
        insert_sql, example_record_ids,
        [](sqlite3_stmt* const stmt, auto const& data_i) { database::Sqlite3Tools::Bind(stmt, 1, data_i); }, db);

    // Load the written record ids
    // NOTE(Jack): Because this query is static and has no dynamic binding parameters we pass in nullptr as the binding
    // function.
    std::string const select_sql{"SELECT record_id FROM example_data_table;"};

    std::vector<int64_t> data;
    database::ExecuteQuery(db, select_sql, nullptr, [&](sqlite3_stmt* stmt) {
        uint64_t const record_id{static_cast<uint64_t>(sqlite3_column_int64(stmt, 0))};

        data.push_back(record_id);
    });

    ASSERT_EQ(std::size(data), std::size(example_record_ids));
    for (size_t i{0}; i < std::size(data); ++i) {
        EXPECT_EQ(data[i], example_record_ids[i]);
    }

    sqlite3_close(db);
}