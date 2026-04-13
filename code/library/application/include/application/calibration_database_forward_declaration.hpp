#pragma once

// TODO ADD NOTE ABOUT HWAT WE ARE DOING WITH THE FOWRARD DECLARATIONS AND WHY THIS IS OUTISDEOF THE NAMESPACE
struct sqlite3;

namespace reprojection::database {

struct SqliteDeleter;

using SqlitePtr = std::unique_ptr<sqlite3, SqliteDeleter>;

}  // namespace reprojection::database
