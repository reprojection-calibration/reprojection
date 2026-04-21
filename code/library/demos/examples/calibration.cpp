#include "application/reprojection_calibration.hpp"

using namespace reprojection;

int main(int argc, char* argv[]) {
    auto const app_args{application::ParseArgs(argc, argv)};
    if (not app_args) {
        return EXIT_FAILURE;
    }

    application::Calibrate(app_args->config, {}, "", app_args->db);

    return EXIT_SUCCESS;
}