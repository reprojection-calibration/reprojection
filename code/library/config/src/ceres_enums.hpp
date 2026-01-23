#pragma once

#include <ceres/types.h>

#include <map>

namespace reprojection::config {

static std::map<std::string, ceres::LinearSolverType> const LinearSolverTypeMap{
    {"DENSE_NORMAL_CHOLESKY", ceres::DENSE_NORMAL_CHOLESKY},
    {"DENSE_QR", ceres::DENSE_QR},
    {"SPARSE_NORMAL_CHOLESKY", ceres::SPARSE_NORMAL_CHOLESKY},
    {"DENSE_SCHUR", ceres::DENSE_SCHUR},
    {"SPARSE_SCHUR", ceres::SPARSE_SCHUR},
    {"ITERATIVE_SCHUR", ceres::ITERATIVE_SCHUR},
    {"CGNR", ceres::CGNR},
};

static std::map<std::string, ceres::PreconditionerType> const PreconditionerTypeMap{
    {"IDENTITY", ceres::IDENTITY},
    {"JACOBI", ceres::JACOBI},
    {"SCHUR_JACOBI", ceres::SCHUR_JACOBI},
    {"SCHUR_POWER_SERIES_EXPANSION", ceres::SCHUR_POWER_SERIES_EXPANSION},
    {"CLUSTER_JACOBI", ceres::CLUSTER_JACOBI},
    {"CLUSTER_TRIDIAGONAL", ceres::CLUSTER_TRIDIAGONAL},
    {"SUBSET", ceres::SUBSET},
};

static std::map<std::string, ceres::VisibilityClusteringType> const VisibilityClusteringTypeMap{
    {"CANONICAL_VIEWS", ceres::CANONICAL_VIEWS},
    {"SINGLE_LINKAGE", ceres::SINGLE_LINKAGE},
};

static std::map<std::string, ceres::SparseLinearAlgebraLibraryType> const SparseLinearAlgebraLibraryTypeMap{
    {"SUITE_SPARSE", ceres::SUITE_SPARSE},
    {"EIGEN_SPARSE", ceres::EIGEN_SPARSE},
    {"ACCELERATE_SPARSE", ceres::ACCELERATE_SPARSE},
    {"CUDA_SPARSE", ceres::CUDA_SPARSE},
    {"NO_SPARSE", ceres::NO_SPARSE},
};

static std::map<std::string, ceres::LinearSolverOrderingType> const LinearSolverOrderingTypeMap{
    {"AMD", ceres::AMD},
    {"NESDIS", ceres::NESDIS},
};

static std::map<std::string, ceres::DenseLinearAlgebraLibraryType> const DenseLinearAlgebraLibraryTypeMap{
    {"EIGEN", ceres::EIGEN},
    {"LAPACK", ceres::LAPACK},
    {"CUDA", ceres::CUDA},
};

static std::map<std::string, ceres::LoggingType> const LoggingTypeMap{
    {"SILENT", ceres::SILENT},
    {"PER_MINIMIZER_ITERATION", ceres::PER_MINIMIZER_ITERATION},
};

static std::map<std::string, ceres::MinimizerType> const MinimizerTypeMap{
    {"LINE_SEARCH", ceres::LINE_SEARCH},
    {"TRUST_REGION", ceres::TRUST_REGION},
};

static std::map<std::string, ceres::LineSearchDirectionType> const LineSearchDirectionMap{
    {"STEEPEST_DESCENT", ceres::STEEPEST_DESCENT},
    {"NONLINEAR_CONJUGATE_GRADIENT", ceres::NONLINEAR_CONJUGATE_GRADIENT},
    {"LBFGS", ceres::LBFGS},
    {"BFGS", ceres::BFGS},
};

static std::map<std::string, ceres::NonlinearConjugateGradientType> const NonlinearConjugateGradientTypeMap{
    {"FLETCHER_REEVES", ceres::FLETCHER_REEVES},
    {"POLAK_RIBIERE", ceres::POLAK_RIBIERE},
    {"HESTENES_STIEFEL", ceres::HESTENES_STIEFEL},
};

static std::map<std::string, ceres::LineSearchType> const LineSearchTypeMap{
    {"ARMIJO", ceres::ARMIJO},
    {"WOLFE", ceres::WOLFE},
};

static std::map<std::string, ceres::TrustRegionStrategyType> const TrustRegionStrategyTypeMap{
    {"LEVENBERG_MARQUARDT", ceres::LEVENBERG_MARQUARDT},
    {"DOGLEG", ceres::DOGLEG},
};

static std::map<std::string, ceres::DoglegType> const DoglegTypeMap{
    {"TRADITIONAL_DOGLEG", ceres::TRADITIONAL_DOGLEG},
    {"SUBSPACE_DOGLEG", ceres::SUBSPACE_DOGLEG},
};

static std::map<std::string, ceres::TerminationType> const TerminationTypeMap{
    {"CONVERGENCE", ceres::CONVERGENCE},   {"NO_CONVERGENCE", ceres::NO_CONVERGENCE}, {"FAILURE", ceres::FAILURE},
    {"USER_SUCCESS", ceres::USER_SUCCESS}, {"USER_FAILURE", ceres::USER_FAILURE},
};

static std::map<std::string, ceres::DumpFormatType> const DumpFormatTypeMap{
    {"CONSOLE", ceres::CONSOLE},
    {"TEXTFILE", ceres::TEXTFILE},

};

static std::map<std::string, ceres::NumericDiffMethodType> const NumericDiffMethodTypeMap{
        {"CENTRAL", ceres::CENTRAL},
        {"FORWARD", ceres::FORWARD},
        {"RIDDERS", ceres::RIDDERS},
    };

static std::map<std::string, ceres::LineSearchInterpolationType> const LineSearchInterpolationTypeMap{
        {"BISECTION", ceres::BISECTION},
        {"QUADRATIC", ceres::QUADRATIC},
        {"CUBIC", ceres::CUBIC},
    };

static std::map<std::string, ceres::CovarianceAlgorithmType> const CovarianceAlgorithmTypeMap{
        {"DENSE_SVD", ceres::DENSE_SVD},
        {"SPARSE_QR", ceres::SPARSE_QR},
    };


}  // namespace reprojection::config