set(LIBRARY_NAME "spline")

set(SRC_FILES
        src/se3_spline.cpp
        src/cubic_spline_c3_init.cpp
        src/sparse_utilities.cpp
        src/spline_initialization.cpp
        src/time_handler.cpp
        src/utilities.cpp
)
set(LINK_LIBRARIES
        # NOTE(Jack): We use raw eigen functions so much in this library that it seems wrong to depend on the transitive
        # eigen inclusion from types_internal or any other library.
        Eigen3::Eigen
        types_internal
)
set(PUBLIC_LINK_LIBRARIES
        geometry
)
AddLibrary()

set(TESTS
        src/cubic_spline_c3_init.test.cpp
        src/sparse_utilities.test.cpp
        test/r3_spline.test.cpp
        test/se3_spline.test.cpp
        test/so3_spline.test.cpp
        test/spline_initialization.test.cpp
        test/spline_state.test.cpp
        test/time_handler.test.cpp
        test/utilities.test.cpp
)
AddTests()