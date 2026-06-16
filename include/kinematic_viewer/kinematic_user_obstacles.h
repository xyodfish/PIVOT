#pragma once

#include "kinematic_viewer/kinematic_collision_monitor.h"
#include "kinematic_viewer/kinematic_runtime_state.h"
#include "rkv/scene.h"

#include <glad/glad.h>

#include <glm/mat4x4.hpp>

namespace kinematic_viewer {

    struct UserObstacleGpuMeshes {
        GLuint box_vao          = 0;
        GLuint box_vbo          = 0;
        GLuint box_ebo          = 0;
        GLsizei box_index_count = 0;

        GLuint sphere_vao          = 0;
        GLuint sphere_vbo          = 0;
        GLuint sphere_ebo          = 0;
        GLsizei sphere_index_count = 0;

        GLuint cylinder_vao          = 0;
        GLuint cylinder_vbo          = 0;
        GLuint cylinder_ebo          = 0;
        GLsizei cylinder_index_count = 0;
    };

    bool InitUserObstacleGpuMeshes(UserObstacleGpuMeshes* gpu);
    void DestroyUserObstacleGpuMeshes(UserObstacleGpuMeshes* gpu);

    void DrawUserObstacles(GLuint mesh_shader, const UserObstacleState& obstacles, const UserObstacleGpuMeshes& gpu, const glm::mat4& view,
                           const glm::mat4& projection);

    void RenderUserObstaclePanel(UserObstacleState* obstacles, bool angle_unit_deg = true);

    bool LoadUserObstaclesFromYaml(const std::string& path, std::vector<UserObstacleItem>* out_items, std::string* errorMessage);
    void UpdateUserObstacleNextSerial(UserObstacleState* obstacles);

    void MergeUserObstaclesIntoCollisionResult(const UserObstacleState& obstacles, const rkv::RobotScene& scene,
                                               float warning_distance_m, float danger_distance_m, CollisionMonitorResult* result);

}  // namespace kinematic_viewer
