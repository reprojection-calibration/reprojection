#include "steps/spline_interpolation.hpp"

namespace reprojection::steps {

std::string SplineInterpolation::CacheKey() const {}

spline::Se3Spline SplineInterpolation::Compute() const {}

spline::Se3Spline SplineInterpolation::Load(SqlitePtr const db) const {}

void SplineInterpolation::Save(spline::Se3Spline const& spline, SqlitePtr const db) const {};

}  // namespace reprojection::steps
