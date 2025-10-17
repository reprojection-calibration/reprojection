macro(AddLibrary)
    add_library(${LIBRARY_NAME} SHARED
            ${SRC_FILES}
    )
    # ERROR(Jack): This separation of of the include/ and src/ folder does NOT work like I want it to. Any other sub
    # directory can include any header file from either the include/ or src/ directory. My intention, and our design
    # here should be that only the headers in the include folder can be used in other sub directories!
    target_include_directories(${LIBRARY_NAME} PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/>
            ${INCLUDE_DIRECTORIES}
    )
    target_link_libraries(${LIBRARY_NAME} PUBLIC
            ${LINK_LIBRARIES}
    )
endmacro()