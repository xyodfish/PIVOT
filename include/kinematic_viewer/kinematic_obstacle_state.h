#pragma once

#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include <string>
#include <vector>

namespace kinematic_viewer {

    struct UserObstacleItem {
        enum class Kind { Box = 0, Sphere = 1, Cylinder = 2 };
        Kind kind = Kind::Box;
        std::string name;
        bool visible = true;
        glm::vec3 color{0.82f, 0.52f, 0.18f};
        glm::vec3 position{0.45f, 0.0f, 0.35f};
        glm::vec3 rpy_deg{0.0f, 0.0f, 0.0f};
        glm::vec3 params{0.4f, 0.4f, 0.5f};
    };

    struct UserObstacleState {
        std::vector<UserObstacleItem> items;
        int selected_index          = -1;
        bool affect_collision       = true;
        int next_serial             = 1;
        bool enable_pose_gizmo      = false;
        int gizmo_operation         = 2;  // 0:translate 1:rotate 2:translate+rotate
        bool gizmo_world_mode       = true;
        float gizmo_size_clip_space = 0.11f;
    };

}  // namespace kinematic_viewer
