#include "steps/config_parsing.hpp"

#include "database/database_write.hpp"

namespace reprojection::steps {

// TODO(Jack): Is this really worth testing?
config::Config ConfigParsing(toml::table const& cfg_table, SqlitePtr const db) {
    config::Config const cfg{config::Config::Parse(cfg_table)};

    // WARN(Jack): At this point in time (30.06.2026) the process is constrained to only handle one camera and one IMU.
    // Therefore, we probably should use "insert or no-op on match" or "delete on mismatch" semantics. But at this time
    // we will just insert the entity ID no matter what. I think this is fine but let's think about it :)
    database::InsertEntity(db, cfg.camera.sensor_name, Entity::Camera);
    if (cfg.imu) {
        database::InsertEntity(db, cfg.imu->sensor_name, Entity::Imu);
        database::InsertEntity(db, Extrinsic::EntityId(cfg.imu->sensor_name, cfg.camera.sensor_name),
                               Entity::Extrinsic);
    }

    // TODO(Jack): Should we insert the config file into the database itself? Right now the application requires a
    // config file to run but it might be nice if it was stored in the database and the user could opt to rerun the
    // calibration using the stored config or even extract the stored config to debug the calibration more.

    return cfg;
}  // LCOV_EXCL_LINE

}  // namespace reprojection::steps
