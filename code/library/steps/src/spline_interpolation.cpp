#include "steps/spline_interpolation.hpp"

#include "caching/cache_keys.hpp"
#include "spline/spline_initialization.hpp"

namespace reprojection::steps {

std::string SplineInterpolation::CacheKey() const { return caching::CacheKey(sensor_name, poses); }

spline::Se3Spline SplineInterpolation::Compute() const {
    // TODO(Jack): Parameterize frequency! Add to cache key.
    spline::Se3Spline const spline{spline::InitializeSe3SplineState(poses, 100)};

    return spline;
}

spline::Se3Spline SplineInterpolation::Load(SqlitePtr const db) const {
    (void)db;

    return spline::Se3Spline{{}, {0, 1}};
}

void SplineInterpolation::Save(spline::Se3Spline const& spline, SqlitePtr const db) const {
    (void)spline;
    (void)db;
}

}  // namespace reprojection::steps
