#pragma once

#include "rkv/scene.h"

#include <glm/vec3.hpp>

#include <string>
#include <vector>

namespace kinematic_viewer {

    struct CollisionPairDistance {
        std::string link_a;
        std::string link_b;
        glm::vec3 point_a        = glm::vec3(0.0f);
        glm::vec3 point_b        = glm::vec3(0.0f);
        float center_distance_m  = 0.0f;
        float surface_distance_m = 0.0f;
    };

    struct MeshDistanceResult {
        float distance_m  = 0.0f;
        glm::vec3 point_a = glm::vec3(0.0f);
        glm::vec3 point_b = glm::vec3(0.0f);
    };

    MeshDistanceResult MinDistanceBetweenTriangleSoups(const std::vector<glm::vec3>& triangles_a,
                                                       const std::vector<glm::vec3>& triangles_b);

    float DistanceAabbToAabb(const glm::vec3& a_min, const glm::vec3& a_max, const glm::vec3& b_min, const glm::vec3& b_max,
                             glm::vec3* out_point_a = nullptr, glm::vec3* out_point_b = nullptr);

    CollisionPairDistance BuildCollisionPairDistanceAabb(const rkv::RobotScene::LinkCollisionProxy& a,
                                                         const rkv::RobotScene::LinkCollisionProxy& b);

    MeshDistanceResult MeshDistanceBetweenLinks(const rkv::RobotScene& scene, const std::string& link_a, const std::string& link_b);

    CollisionPairDistance BuildCollisionPairDistance(const rkv::RobotScene::LinkCollisionProxy& a,
                                                     const rkv::RobotScene::LinkCollisionProxy& b,
                                                     const rkv::RobotScene* scene_for_mesh = nullptr, bool use_mesh = false);

    CollisionPairDistance RefineCollisionPairDistanceWithMesh(const rkv::RobotScene& scene, const CollisionPairDistance& aabb_distance);

}  // namespace kinematic_viewer
