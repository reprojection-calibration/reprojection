#pragma once

#include <ceres/ceres.h>

namespace reprojection_calibration::optimization_architecture {

struct OneParameterCostFunction {
    explicit OneParameterCostFunction(double const data) : data_{data} {}

    template <typename T>
    bool operator()(const T* const x, T* const residual) const {
        residual[0] = data_ - x[0];

        return true;
    }

    static ceres::CostFunction* Create(double const data) {
        return new ceres::AutoDiffCostFunction<OneParameterCostFunction, 1, 1>(new OneParameterCostFunction(data));
    }

    double data_;
};

struct TwoParameterCostFunction {
    explicit TwoParameterCostFunction(double const data) : data_{data} {}

    template <typename T>
    bool operator()(const T* const x, T* const residual) const {
        residual[0] = data_ - x[0] - x[1];

        return true;
    }

    static ceres::CostFunction* Create(double const data) {
        return new ceres::AutoDiffCostFunction<TwoParameterCostFunction, 1, 2>(new TwoParameterCostFunction(data));
    }

    double data_;
};

}  // namespace reprojection_calibration::optimization_architecture