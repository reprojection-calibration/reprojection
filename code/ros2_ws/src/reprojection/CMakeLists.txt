cmake_minimum_required(VERSION 3.8)
project(reprojection)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(reprojection_app src/reprojection_app.cpp src/reprojection.cpp)
target_include_directories(reprojection_app PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(reprojection_app
        rclcpp
)

ament_package()
