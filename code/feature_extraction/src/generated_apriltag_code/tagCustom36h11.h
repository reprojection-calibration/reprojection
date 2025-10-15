#ifndef _TAGCustom36H11
#define _TAGCustom36H11

// WARN(Jack): The relative path "apriltag/" needs to be added here because we are using the external installed package!
// If you look at the generated code from the source repo (https://github.com/AprilRobotics/apriltag-generation) the
// include will just be #include "apriltag.h". Code was also clang formatted!
#include <apriltag/apriltag.h>

#ifdef __cplusplus
extern "C" {
#endif

apriltag_family_t* tagCustom36h11_create();
void tagCustom36h11_destroy(apriltag_family_t* tf);

#ifdef __cplusplus
}
#endif

#endif
