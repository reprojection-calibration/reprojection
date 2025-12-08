#include <gtest/gtest.h>
#include <sqlite3.h>

#include <filesystem>
#include <string>

namespace reprojection::database {

enum class SqliteFlag {
    Ok = SQLITE_OK,
    OpenReadOnly = SQLITE_OPEN_READONLY,
    OpenReadWrite = SQLITE_OPEN_READWRITE,
    OpenCreate = SQLITE_OPEN_CREATE
};

struct ImuData {
    double angular_velocity[3];
    double linear_acceleration[3];
};

struct Sqlite3Tools {
    [[nodiscard]] static bool Execute(std::string const& sql_statement, sqlite3* const db) {
        char* errror_msg{0};
        int const code{sqlite3_exec(db, sql_statement.c_str(), nullptr, nullptr, &errror_msg)};

        if (code != static_cast<int>(SqliteFlag::Ok)) {
            std::cerr << "SQL error: " << errror_msg << std::endl;
            sqlite3_free(errror_msg);  // WARN(Jack): Violating RAII here! Should wrap errror_msg with class.

            return false;
        }

        return true;
    }
};

// NOTE(Jack): Technically which database we use DOES NOT matter to the calibration process (it is an implementation
// detail only)! However instead of starting with a pure virtual interface base class to define the interface, we will
// start with a concrete implementation and if we need generalization later we will refactor.
class CalibrationDatabase {
   public:
    CalibrationDatabase(std::string const& db_path, bool const create, bool const read_only = false) {
        if (create and read_only) {
            throw std::runtime_error(
                "You requested to open a database object with both options 'create' and 'read_only' true. This is "
                "an invalid combination as creating a database requires writing to it!");
        }

        // TODO(Jack): Consider using sqlite3_errcode for better terminal output https://sqlite.org/c3ref/errcode.html
        int code;
        if (create) {
            // WARN(Jack): Shoudl it be an error if create is true and the database already exists. Is that a problem?
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

    [[nodiscard]] bool AddImuData(uint64_t const timestamp_ns, std::string const& sensor_name, ImuData const& data) {
        // TODO(Jack): This is hacky! We need to find a principle way to create all data tables, not recreate it
        // everytime we want to add data!
        // TODO(Jack): Is there any problem we will have storing our time type (uint64t) here as a signed integer type?
        // TODO(Jack): Should index the sensor by an integer id or by a string?
        std::string const init_imu_table{
            "CREATE TABLE IF NOT EXISTS imu_data ("
            "timestamp_ns INTEGER NOT NULL, "
            "sensor_name TEXT NOT NULL, "
            "omega_x REAL NOT NULL, "
            "omega_y REAL NOT NULL, "
            "omega_z REAL NOT NULL, "
            "ax REAL NOT NULL, "
            "ay REAL NOT NULL, "
            "az REAL NOT NULL, "
            "PRIMARY KEY (timestamp_ns, sensor_name)"
            ");"};
        bool success{Sqlite3Tools::Execute(init_imu_table, db_)};
        if (not success) {
            return false;
        }

        (void)data;
        std::string const add_imu_row{
            "INSERT INTO imu_data (timestamp_ns, sensor_name, omega_x, omega_y, omega_z, ax, ay, az) "
            "VALUES(" +
            std::to_string(timestamp_ns) + ", '" + sensor_name + "', " + std::to_string(data.angular_velocity[0]) +
            ", " + std::to_string(data.angular_velocity[1]) + ", " + std::to_string(data.angular_velocity[2]) + ", " +
            std::to_string(data.linear_acceleration[0]) + ", " + std::to_string(data.linear_acceleration[1]) + ", " +
            std::to_string(data.linear_acceleration[2]) + ");"};

        return Sqlite3Tools::Execute(add_imu_row, db_);
    }

   private:
    sqlite3* db_;
};

}  // namespace reprojection::database

using namespace reprojection;

// Test fixture used to facilitate isolated filesystem state. This is useful when testing database creation to prevent
// artefacts from previous or parallel testing interfering with the system currently under test.
class TempFolder : public ::testing::Test {
   protected:
    void SetUp() override { std::filesystem::create_directories(database_path_); }

    void TearDown() override { std::filesystem::remove_all(database_path_); }

    std::string database_path_{"sandbox"};
};

TEST_F(TempFolder, TestAddImuData) {
    std::string const record{database_path_ + "/record_hhh.db3"};
    database::CalibrationDatabase db{record, true};

    bool success{db.AddImuData(0, "/imu/polaris/123", {})};
    EXPECT_TRUE(success);
    success = db.AddImuData(1, "/imu/polaris/123", {});
    EXPECT_TRUE(success);

    // Add second sensors data with same timestamp as a preexisting record - works because we use a compound primary key
    // (timestamp_ns, sensor_name)
    success = db.AddImuData(0, "/imu/polaris/456", {});
    EXPECT_TRUE(success);

    // Add a repeated record - this fails because the primary key must always be unique!
    success = db.AddImuData(0, "/imu/polaris/456", {});
    EXPECT_FALSE(success);
}

TEST_F(TempFolder, TestCreate) {
    std::string const record{database_path_ + "/record_xxx.db3"};

    // Cannot create a database when read_only is true - creating a database requires writing to it!
    EXPECT_THROW(database::CalibrationDatabase(record, true, true), std::runtime_error);

    // Cannot open a non-existent database
    EXPECT_THROW(database::CalibrationDatabase(record, false), std::runtime_error);

    // Create a database (found on the filesystem) and then check that we can open it
    database::CalibrationDatabase(record, true);
    EXPECT_NO_THROW(database::CalibrationDatabase(record, false));
}

TEST_F(TempFolder, TestReadWrite) {
    std::string const record{database_path_ + "/record_yyy.db3"};
    database::CalibrationDatabase(record, true);

    EXPECT_NO_THROW(database::CalibrationDatabase(record, false, false));
}

TEST_F(TempFolder, TestReadOnly) {
    std::string const record{database_path_ + "/record_zzz.db3"};
    database::CalibrationDatabase(record, true);

    EXPECT_NO_THROW(database::CalibrationDatabase(record, false, true));
}