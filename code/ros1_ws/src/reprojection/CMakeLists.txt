cmake_minimum_required(VERSION 3.0.2)
project(reprojection)

add_compile_options(-std=c++20)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
)

catkin_package()

add_executable(reprojection_app src/reprojection_app.cpp src/reprojection.cpp)
target_include_directories(reprojection_app PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        ${catkin_INCLUDE_DIRS}
)
target_link_libraries(reprojection_app
        ${catkin_LIBRARIES}
)


# NOTE(Jack): Ros would have us use catkin test stuff here, but I could not get this simply to work with the CLion
# integration, I think because catkin wants to generate the tests in the workspace folders but we want to use the CLion
# cmake-build-* folder instead. For now this solution works as a stop gap.
find_package(GTest REQUIRED)

if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(reprojection_test test/reprojection.test.cpp)
endif()

# also allow pure CMake tests
#add_executable(reprojection_test test/reprojection.test.cpp)
target_include_directories(reprojection_test PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        ${catkin_INCLUDE_DIRS}
)
target_link_libraries(reprojection_test GTest::Main ${catkin_LIBRARIES})
gtest_discover_tests(reprojection_test)




