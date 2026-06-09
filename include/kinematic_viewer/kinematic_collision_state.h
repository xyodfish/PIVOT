#pragma once

#include <glm/vec3.hpp>

#include <string>

namespace kinematic_viewer {

    struct CollisionMonitorState {
        bool enable                 = false;
        bool ignore_same_link       = true;
        bool ignore_parent_child    = true;
        bool show_closest_pair_line = true;
        float warning_distance_m    = 0.08f;
        float danger_distance_m     = 0.03f;
        bool has_valid_distance     = false;
        std::string nearest_link_a;
        std::string nearest_link_b;
        float nearest_surface_distance_m = 0.0f;
        float nearest_center_distance_m  = 0.0f;
        glm::vec3 nearest_point_a        = glm::vec3(0.0f);
        glm::vec3 nearest_point_b        = glm::vec3(0.0f);
        int evaluated_pair_count         = 0;
    };

}  // namespace kinematic_viewer
