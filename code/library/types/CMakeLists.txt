set(LIBRARY_NAME "types")

add_library(${LIBRARY_NAME} INTERFACE)
target_include_directories(${LIBRARY_NAME} INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
        $<INSTALL_INTERFACE:include>
)

target_compile_options(${LIBRARY_NAME} INTERFACE
        $<$<COMPILE_LANGUAGE:CXX>:-Wno-deprecated-enum-enum-conversion>
)

install(TARGETS ${LIBRARY_NAME}
        EXPORT reprojectionTargets
)