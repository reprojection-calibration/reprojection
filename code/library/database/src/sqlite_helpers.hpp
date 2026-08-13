#pragma once

#include <sqlite3.h>

#include <Eigen/Dense>
#include <span>

#include "database/sqlite_exception.hpp"

namespace reprojection::database {

class SqlStatement {
   public:
    SqlStatement(sqlite3* const db, char const* const sql);

    ~SqlStatement();

    sqlite3_stmt* stmt_{nullptr};
};

struct SqlTransaction {
    explicit SqlTransaction(sqlite3* const db);

    ~SqlTransaction();

   private:
    sqlite3* db_;
};

void Bind(sqlite3_stmt* const stmt, int const index, std::string_view value);

void Bind(sqlite3_stmt* const stmt, int const index, int64_t const value);

void Bind(sqlite3_stmt* const stmt, int const index, std::size_t const value);

void Bind(sqlite3_stmt* const stmt, int const index, double const value);

void BindNull(sqlite3_stmt* const stmt, int const index);

void BindBlob(sqlite3_stmt* const stmt, int const index, std::span<std::byte const> const& blob);

template <typename T>
    requires(T::ColsAtCompileTime == 1)
void BindEigenColumn(sqlite3_stmt* const stmt, int const start_idx, Eigen::DenseBase<T> const& v) {
    for (Eigen::Index i{0}; i < v.size(); ++i) {
        Bind(stmt, start_idx + static_cast<int>(i), v.derived()(i));
    }
}

template <int N>
Eigen::Array<double, N, 1> ReadEigenColumn(sqlite3_stmt* const stmt, int const start_idx) {
    Eigen::Array<double, N, 1> result;
    for (int i{0}; i < N; ++i) {
        result(i) = sqlite3_column_double(stmt, start_idx + i);
    }

    return result;
}

bool StepRow(sqlite3_stmt* const stmt);

std::span<const std::byte> SqliteBlob(sqlite3_stmt* const stmt, int const col);

// Used for cases that do not require dynamic binding.
void ExecuteStatement(std::string_view sql, sqlite3* const db);

template <typename Binder>
void ExecuteStatement(std::string_view sql, Binder&& binder, sqlite3* const db) {
    SqlStatement stmt{db, std::string(sql).c_str()};

    try {
        binder(stmt.stmt_);
    } catch (...) {  // LCOV_EXCL_LINE
        // TODO(Jack): It think it can very well be that any error thrown from bind is actually not 100% sqlite related,
        //  but actually due a error in the user code. Therefore it might be a mistake here to throw away the thrown
        //  error and replace it here with a database centric error. Think about also throwing the original error too!
        throw SqliteException(db, stmt.stmt_);  // LCOV_EXCL_LINE
    }

    if (sqlite3_step(stmt.stmt_) != SQLITE_DONE) {
        throw SqliteException(db, stmt.stmt_);  // LCOV_EXCL_LINE
    }
}

// TODO(Jack): Can we use concepts here to enforce some properties on Container and Binder?
template <typename Container, typename Binder>
void BatchExecuteStatement(std::string_view sql, Container const& data, Binder&& binder, sqlite3* const db) {
    for (SqlTransaction const transaction{db}; auto const& data_i : data) {
        ExecuteStatement(sql, [&](sqlite3_stmt* stmt) { binder(stmt, data_i); }, db);
    }
}

template <typename Binder, typename RowFunc>
void ExecuteQuery(sqlite3* const db, std::string_view sql, Binder&& binder, RowFunc&& on_row) {
    SqlStatement stmt{db, std::string(sql).data()};

    try {
        // NOTE(Jack): If the sql query statement does not use any dynamic binding (i.e. we want to perform a static
        // operation like creating a table), then we do not need to call the binder. Therefore, this code lets the user
        // pass in a nullptr for the binder, and it will then not execute any binding call. If this is actually possible
        // for people to understand is not clear at this time :)
        if constexpr (not std::is_same_v<std::decay_t<Binder>, std::nullptr_t>) {
            binder(stmt.stmt_);
        }
    } catch (...) {                                               // LCOV_EXCL_LINE
        std::throw_with_nested(SqliteException(db, stmt.stmt_));  // LCOV_EXCL_LINE
    }

    try {
        while (StepRow(stmt.stmt_)) {
            on_row(stmt.stmt_);
        }
    } catch (...) {                                               // LCOV_EXCL_LINE
        std::throw_with_nested(SqliteException(db, stmt.stmt_));  // LCOV_EXCL_LINE
    }
}

}  // namespace reprojection::database