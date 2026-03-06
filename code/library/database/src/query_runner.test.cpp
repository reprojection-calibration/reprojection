#include "query_runner.hpp"

#include <gtest/gtest.h>

#include <string>

#include "test_fixture.hpp"

using namespace reprojection;

TEST_F(SqliteTestFixture, TestExecuteQuery) {
    CreateExampleTable();
    InsertRecordIds();

    // Load the written record ids
    // NOTE(Jack): Because this query is static and has no dynamic binding parameters we pass in nullptr as the binding
    // function.
    std::string const select_sql{"SELECT record_id FROM example_data_table;"};

    std::vector<int64_t> data;
    database::ExecuteQuery(db, select_sql, nullptr, [&data](sqlite3_stmt* stmt) {
        uint64_t const record_id{static_cast<uint64_t>(sqlite3_column_int64(stmt, 0))};
        data.push_back(record_id);
    });

    ASSERT_EQ(std::size(data), std::size(example_record_ids));
    for (size_t i{0}; i < std::size(data); ++i) {
        EXPECT_EQ(data[i], example_record_ids[i]);
    }
}