set(LIBRARY_NAME "video_capture")

set(SRC_FILES
        src/video_capture.cpp
)

set(LINK_LIBRARIES
        ${OpenCV_LIBS}
)
set(INSTALL_SO false)
AddLibrary()

set(TESTS
        test/video_capture.test.cpp

)
AddTests()
