#pragma once

#include <sqlite3.h>

namespace reprojection::database {

enum class SqliteFlag {
    Done = SQLITE_DONE,
    Ok = SQLITE_OK,
    OpenReadOnly = SQLITE_OPEN_READONLY,
    OpenReadWrite = SQLITE_OPEN_READWRITE,
    OpenCreate = SQLITE_OPEN_CREATE,
    Row = SQLITE_ROW
};

}