#include "batch_insert.hpp"

#include <gtest/gtest.h>

#include <string>

#include "sqlite3_helpers.hpp"

using namespace reprojection;

// TODO(Jack): Use test fixture!

TEST(DatabaseBatchInsert, TestBatchInsert) {
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

    EXPECT_NO_THROW(database::BatchInsert(
        insert_sql, example_record_ids,
        [](sqlite3_stmt* const stmt, auto const& data_i) { database::Sqlite3Tools::Bind(stmt, 1, data_i); }, db));

    sqlite3_close(db);
}