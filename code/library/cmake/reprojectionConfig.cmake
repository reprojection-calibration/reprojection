include(CMakeFindDependencyMacro)

# TODO(Jack): Get rid of these by using $<BUILD_INTERFACE:...> and $<INSTALL_INTERFACE:...> for the linked libraries. I
# am not really sure that is the right option, but I think having these here is a hack just to get rid of a cmake error
# I had. One glaring problem here is that Eigen3 is header only and also not found in the public app interface, but
# because it was declared in some cmakelists as a public dependency it needs to be included here. Where is a cmake
# wizard to look at this problem? :)
find_dependency(Ceres REQUIRED)
find_dependency(Eigen3 REQUIRED)
find_dependency(SQLite3 REQUIRED)
find_dependency(spdlog REQUIRED)

include("${CMAKE_CURRENT_LIST_DIR}/reprojectionTargets.cmake")