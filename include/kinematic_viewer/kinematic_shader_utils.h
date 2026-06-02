#pragma once

#include <glad/glad.h>

namespace kinematic_viewer {

    GLuint createKinematicMeshProgram();
    GLuint createKinematicLineProgram();
    GLuint createKinematicPointProgram();

}  // namespace kinematic_viewer
