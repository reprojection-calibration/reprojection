set(LIBRARY_NAME "image_viewer")

set(SRC_FILES
        src/image_viewer.cpp
        src/keyboard_input_parsing.cpp
)
set(INCLUDE_DIRECTORIES
        ${OpenCV_INCLUDE_DIRS}
)
set(LINK_LIBRARIES
        ${OpenCV_LIBS}
)
AddLibrary()

set(TESTS
        test/image_viewer.test.cpp

)
AddTests()
