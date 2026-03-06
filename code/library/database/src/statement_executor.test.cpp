#include "statement_executor.hpp"

#include <gtest/gtest.h>

#include <string>

#include "sqlite3_helpers.hpp"
#include "test_fixture.hpp"

using namespace reprojection;

TEST_F(SqliteTestFixture, TestExecute) {
    // Test the "no binding" override used in this fixture method.
    EXPECT_NO_THROW(CreateExampleTable());

    // Throws because you cannot create a duplicated table!
    EXPECT_THROW(CreateExampleTable(), std::runtime_error);

    // Test with binding lambda.
    int64_t const example_record_id{0};
    EXPECT_NO_THROW(database::ExecuteStatement(
        insert_sql, [](sqlite3_stmt* const stmt) { database::Sqlite3Tools::Bind(stmt, 1, example_record_id); }, db));

    // Throws on the bind because there is only one value to bind but we select the 2nd index.
    EXPECT_THROW(
        database::ExecuteStatement(
            insert_sql, [](sqlite3_stmt* const stmt) { database::Sqlite3Tools::Bind(stmt, 2, example_record_id); }, db),
        std::runtime_error);
}

TEST_F(SqliteTestFixture, TestBatchExecuteStatement) {
    CreateExampleTable();

    EXPECT_NO_THROW(InsertRecordIds());
}