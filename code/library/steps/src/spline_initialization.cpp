#include "caching/cache_keys.hpp"
#include "database/database_read.hpp"
#include "database/database_write.hpp"
#include "spline/spline_initialization.hpp"
#include "steps/spline_initialization.hpp"

namespace reprojection::steps {

std::string SplineInitialization::CacheKey() const { return caching::CacheKey(sensor_name, poses); }

spline::Se3Spline SplineInitialization::Compute() const {
    // TODO(Jack): Parameterize frequency! Add to cache key probably?
    spline::Se3Spline const spline{spline::InitializeSe3SplineState(poses, 100)};

    return spline;
}

spline::Se3Spline SplineInitialization::Load(SqlitePtr const db) const {
    auto const control_points{database::ReadSplineControlPoints(db, CalibrationStep::SplineInterpolation, sensor_name)};
    auto const time_handler{database::ReadSplineTimeHandler(db, CalibrationStep::SplineInterpolation, sensor_name)};

    if (not time_handler) {
        std::cout << "WE NEED AN ERROR HANDLING STRATEGY! SplineInterpolation::Load()" << std::endl;
    }

    return spline::Se3Spline{control_points, *time_handler};
}

void SplineInitialization::Save(spline::Se3Spline const& spline, SqlitePtr const db) const {
    database::WriteToDb(spline.ControlPoints(), step_type, sensor_name, db);
    database::WriteToDb(spline.GetTimeHandler(), step_type, sensor_name, db);
}

}  // namespace reprojection::steps
