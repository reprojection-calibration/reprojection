#include <gtest/gtest.h>
#include <sqlite3.h>

namespace reprojection::database {

enum class SqliteFlag {
    OpenReadOnly = SQLITE_OPEN_READONLY,
    OpenReadWrite = SQLITE_OPEN_READWRITE,
    OpenCreate = SQLITE_OPEN_CREATE
};

// NOTE(Jack): Technically which database we use DOES NOT matter to the calibration process (it is an implementation
// detail only)! However instead of starting with a pure virtual interface base class to define the interface, we will
// start with a concrete implementation and if we need generalization later we will refactor.
class CalibrationDatabase {
   public:
    CalibrationDatabase(std::string const& db_path, bool const create, bool const read_only = false) {
        int code;
        if (create) {
            code = sqlite3_open_v2(
                db_path.c_str(), &db_,
                static_cast<int>(SqliteFlag::OpenReadWrite) | static_cast<int>(SqliteFlag::OpenCreate), nullptr);
        } else if (read_only) {
            code = sqlite3_open_v2(db_path.c_str(), &db_, static_cast<int>(SqliteFlag::OpenReadOnly), nullptr);
        } else {
            code = sqlite3_open_v2(db_path.c_str(), &db_, static_cast<int>(SqliteFlag::OpenReadWrite), nullptr);
        }

        if (code != 0) {
            sqlite3_close(db_);
            throw std::runtime_error("Attempted to open database at path - " + db_path + " - but was unsuccessful");
        }
    }

    ~CalibrationDatabase() { sqlite3_close(db_); }

   private:
    sqlite3* db_;
};

}  // namespace reprojection::database

using namespace reprojection;

TEST(DatabaseDatabase, TestWhat) {
    database::CalibrationDatabase const db{"record1.d", false, false};

    EXPECT_EQ(1, 2);
}