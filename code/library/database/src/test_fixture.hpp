#include <gtest/gtest.h>

#include "database/calibration_database.hpp"
#include "types/io.hpp"

#include "statement_executor.hpp"

class SqliteTestFixture : public ::testing::Test {
   protected:
    // cppcheck-suppress unusedFunction
    void SetUp() override {
        sqlite3* raw_db{nullptr};
        ASSERT_EQ(sqlite3_open(":memory:", &raw_db), SQLITE_OK);

        db.reset(raw_db, reprojection::database::SqliteDeleter());
    }

    void CreateExampleTable() const {
        std::string const sql{
            "CREATE TABLE example_data_table ("
            "record_id INTEGER"
            ");"};

        reprojection::database::ExecuteStatement(sql, db);
    }

    void InsertRecordIds() const {
        reprojection::database::BatchExecuteStatement(
            insert_sql, example_record_ids,
            [](sqlite3_stmt* const stmt, auto const& data_i) {
                reprojection::database::Sqlite3Tools::Bind(stmt, 1, data_i);
            },
            db);
    }

    reprojection::SqlitePtr db{nullptr};
    std::string const insert_sql{"INSERT INTO example_data_table (record_id) VALUES (?);"};
    std::vector<int64_t> const example_record_ids{0, 1, 2, 3, 4, 5};
};
