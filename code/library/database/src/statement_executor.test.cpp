#include "statement_executor.hpp"

#include <gtest/gtest.h>

#include <string>

#include "sqlite3_helpers.hpp"

using namespace reprojection;

TEST(DatabaseStatementExecutors, TestExecute) {
    sqlite3* db;
    sqlite3_open(":memory:", &db);

    // Test the "no binding" override.
    std::string const data_table_sql{
        "CREATE TABLE example_data_table ("
        "record_id INTEGER "
        ");"};
    EXPECT_NO_THROW(database::ExecuteStatement(data_table_sql, db));

    // Throws because you cannot create a duplicated table!
    EXPECT_THROW(database::ExecuteStatement(data_table_sql, db), std::runtime_error);

    // Test with binding lambda.
    std::string const insert_sql{
        "INSERT INTO example_data_table (record_id) "
        "VALUES (?);"};
    int64_t const example_record_id{0};
    EXPECT_NO_THROW(database::ExecuteStatement(
        insert_sql,
        [example_record_id](sqlite3_stmt* const stmt) { database::Sqlite3Tools::Bind(stmt, 1, example_record_id); },
        db));

    // Throws on the bind because there is only one value to bind but we select the 2nd index.
    EXPECT_THROW(
        database::ExecuteStatement(
            insert_sql,
            [example_record_id](sqlite3_stmt* const stmt) { database::Sqlite3Tools::Bind(stmt, 2, example_record_id); },
            db),
        std::runtime_error);

    sqlite3_close(db);
}