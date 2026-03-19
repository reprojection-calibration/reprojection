set(LIBRARY_NAME "application")

find_package(Eigen3 REQUIRED)

set(SRC_FILES
        src/reprojection_calibration.cpp
        src/steps.cpp
)

set(LINK_LIBRARIES
        Eigen3::Eigen
        caching
        calibration
        database
        optimization
        projection_functions # Introduced due to adding EstimatePoseViaPinholePnP to the linear_pose_init public header interface.
        spline # Required because of the EstimateCameraImuRotationAndGravity() interface!
        testing_mocks
        testing_utilities
        types
)
AddLibrary()

set(TESTS
        src/step_runner.test.cpp
        src/steps.test.cpp
)
AddTests()

include(GNUInstallDirs)

install(TARGETS application
        caching
        calibration
        database
        database_proto
        eigen_utilities
        geometry
        optimization
        pnp
        projection_functions
        spline
        testing_mocks
        testing_utilities
        types
        EXPORT applicationTargets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT applicationTargets
        FILE applicationTargets.cmake
        NAMESPACE reprojection::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/application
)
