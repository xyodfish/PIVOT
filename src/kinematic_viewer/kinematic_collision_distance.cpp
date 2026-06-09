#include "kinematic_viewer/kinematic_collision_distance.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace kinematic_viewer {
    namespace {

        constexpr float kDistanceEpsilon = 1e-8f;

        float Clamp01(float value) {
            return std::clamp(value, 0.0f, 1.0f);
        }

        glm::vec3 ClosestPointOnSegment(const glm::vec3& p, const glm::vec3& a, const glm::vec3& b) {
            const glm::vec3 ab = b - a;
            const float ab_len2 = glm::dot(ab, ab);
            if (ab_len2 <= kDistanceEpsilon) {
                return a;
            }
            const float t = Clamp01(glm::dot(p - a, ab) / ab_len2);
            return a + t * ab;
        }

        float PointTriangleDistance(const glm::vec3& p, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c,
                                    glm::vec3* closest_on_triangle) {
            const glm::vec3 ab = b - a;
            const glm::vec3 ac = c - a;
            const glm::vec3 ap = p - a;

            const float d1 = glm::dot(ab, ap);
            const float d2 = glm::dot(ac, ap);
            if (d1 <= 0.0f && d2 <= 0.0f) {
                if (closest_on_triangle != nullptr) {
                    *closest_on_triangle = a;
                }
                return glm::length(p - a);
            }

            const glm::vec3 bp = p - b;
            const float d3     = glm::dot(ab, bp);
            const float d4     = glm::dot(ac, bp);
            if (d3 >= 0.0f && d4 <= d3) {
                if (closest_on_triangle != nullptr) {
                    *closest_on_triangle = b;
                }
                return glm::length(p - b);
            }

            const float vc = d1 * d4 - d3 * d2;
            if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
                const float v = d1 / (d1 - d3 + kDistanceEpsilon);
                const glm::vec3 q = a + v * ab;
                if (closest_on_triangle != nullptr) {
                    *closest_on_triangle = q;
                }
                return glm::length(p - q);
            }

            const glm::vec3 cp = p - c;
            const float d5     = glm::dot(ab, cp);
            const float d6     = glm::dot(ac, cp);
            if (d6 >= 0.0f && d5 <= d6) {
                if (closest_on_triangle != nullptr) {
                    *closest_on_triangle = c;
                }
                return glm::length(p - c);
            }

            const float vb = d5 * d2 - d1 * d6;
            if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
                const float w = d2 / (d2 - d6 + kDistanceEpsilon);
                const glm::vec3 q = a + w * ac;
                if (closest_on_triangle != nullptr) {
                    *closest_on_triangle = q;
                }
                return glm::length(p - q);
            }

            const float va = d3 * d6 - d5 * d4;
            if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
                const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6) + kDistanceEpsilon);
                const glm::vec3 q = b + w * (c - b);
                if (closest_on_triangle != nullptr) {
                    *closest_on_triangle = q;
                }
                return glm::length(p - q);
            }

            const float denom = 1.0f / (va + vb + vc + kDistanceEpsilon);
            const float v     = vb * denom;
            const float w     = vc * denom;
            const glm::vec3 q = a + ab * v + ac * w;
            if (closest_on_triangle != nullptr) {
                *closest_on_triangle = q;
            }
            return glm::length(p - q);
        }

        float SegmentSegmentDistance(const glm::vec3& p1, const glm::vec3& q1, const glm::vec3& p2, const glm::vec3& q2,
                                     glm::vec3* closest_on_first, glm::vec3* closest_on_second) {
            const glm::vec3 d1 = q1 - p1;
            const glm::vec3 d2 = q2 - p2;
            const glm::vec3 r  = p1 - p2;
            const float a      = glm::dot(d1, d1);
            const float e      = glm::dot(d2, d2);
            const float f      = glm::dot(d2, r);

            float s = 0.0f;
            float t = 0.0f;

            if (a <= kDistanceEpsilon && e <= kDistanceEpsilon) {
                if (closest_on_first != nullptr) {
                    *closest_on_first = p1;
                }
                if (closest_on_second != nullptr) {
                    *closest_on_second = p2;
                }
                return glm::length(p1 - p2);
            }

            if (a <= kDistanceEpsilon) {
                s = 0.0f;
                t = Clamp01(f / e);
            } else {
                const float c = glm::dot(d1, r);
                if (e <= kDistanceEpsilon) {
                    t = 0.0f;
                    s = Clamp01(-c / a);
                } else {
                    const float b     = glm::dot(d1, d2);
                    const float denom = a * e - b * b;
                    if (denom > kDistanceEpsilon) {
                        s = Clamp01((b * f - c * e) / denom);
                    } else {
                        s = 0.0f;
                    }
                    t = (b * s + f) / e;
                    if (t < 0.0f) {
                        t = 0.0f;
                        s = Clamp01(-c / a);
                    } else if (t > 1.0f) {
                        t = 1.0f;
                        s = Clamp01((b - c) / a);
                    }
                }
            }

            const glm::vec3 c1 = p1 + d1 * s;
            const glm::vec3 c2 = p2 + d2 * t;
            if (closest_on_first != nullptr) {
                *closest_on_first = c1;
            }
            if (closest_on_second != nullptr) {
                *closest_on_second = c2;
            }
            return glm::length(c1 - c2);
        }

        float TriangleTriangleDistance(const glm::vec3& a0, const glm::vec3& a1, const glm::vec3& a2, const glm::vec3& b0,
                                       const glm::vec3& b1, const glm::vec3& b2, glm::vec3* out_a, glm::vec3* out_b) {
            float best = std::numeric_limits<float>::infinity();
            glm::vec3 best_a(0.0f);
            glm::vec3 best_b(0.0f);

            auto consider = [&](float distance, const glm::vec3& point_a, const glm::vec3& point_b) {
                if (distance < best) {
                    best   = distance;
                    best_a = point_a;
                    best_b = point_b;
                }
            };

            const glm::vec3 tri_a[3] = {a0, a1, a2};
            const glm::vec3 tri_b[3] = {b0, b1, b2};

            for (const glm::vec3& vertex : tri_a) {
                glm::vec3 closest_on_b(0.0f);
                const float distance = PointTriangleDistance(vertex, b0, b1, b2, &closest_on_b);
                consider(distance, vertex, closest_on_b);
            }
            for (const glm::vec3& vertex : tri_b) {
                glm::vec3 closest_on_a(0.0f);
                const float distance = PointTriangleDistance(vertex, a0, a1, a2, &closest_on_a);
                consider(distance, closest_on_a, vertex);
            }

            const glm::vec3 edges_a[3][2] = {{a0, a1}, {a1, a2}, {a2, a0}};
            const glm::vec3 edges_b[3][2] = {{b0, b1}, {b1, b2}, {b2, b0}};
            for (const auto& edge_a : edges_a) {
                for (const auto& edge_b : edges_b) {
                    glm::vec3 closest_a(0.0f);
                    glm::vec3 closest_b(0.0f);
                    const float distance =
                        SegmentSegmentDistance(edge_a[0], edge_a[1], edge_b[0], edge_b[1], &closest_a, &closest_b);
                    consider(distance, closest_a, closest_b);
                }
            }

            if (out_a != nullptr) {
                *out_a = best_a;
            }
            if (out_b != nullptr) {
                *out_b = best_b;
            }
            return best;
        }

        bool TriangleSoupValid(const std::vector<glm::vec3>& triangles) {
            return triangles.size() >= 3 && (triangles.size() % 3) == 0;
        }

    }  // namespace

    float DistanceAabbToAabb(const glm::vec3& a_min, const glm::vec3& a_max, const glm::vec3& b_min, const glm::vec3& b_max,
                             glm::vec3* out_point_a, glm::vec3* out_point_b) {
        const glm::vec3 a_center = 0.5f * (a_min + a_max);
        const glm::vec3 b_center = 0.5f * (b_min + b_max);

        glm::vec3 point_a = a_center;
        glm::vec3 point_b = b_center;
        glm::vec3 gap(0.0f);
        for (int axis = 0; axis < 3; ++axis) {
            if (a_max[axis] < b_min[axis]) {
                point_a[axis] = a_max[axis];
                point_b[axis] = b_min[axis];
                gap[axis]     = b_min[axis] - a_max[axis];
            } else if (b_max[axis] < a_min[axis]) {
                point_a[axis] = a_min[axis];
                point_b[axis] = b_max[axis];
                gap[axis]     = a_min[axis] - b_max[axis];
            } else {
                point_b[axis] = std::clamp(point_a[axis], b_min[axis], b_max[axis]);
                point_a[axis] = std::clamp(point_b[axis], a_min[axis], a_max[axis]);
                point_b[axis] = std::clamp(point_a[axis], b_min[axis], b_max[axis]);
            }
        }

        if (out_point_a != nullptr) {
            *out_point_a = point_a;
        }
        if (out_point_b != nullptr) {
            *out_point_b = point_b;
        }
        return glm::length(gap);
    }

    MeshDistanceResult MinDistanceBetweenTriangleSoups(const std::vector<glm::vec3>& triangles_a,
                                                       const std::vector<glm::vec3>& triangles_b) {
        MeshDistanceResult result;
        if (!TriangleSoupValid(triangles_a) || !TriangleSoupValid(triangles_b)) {
            result.distance_m = std::numeric_limits<float>::infinity();
            return result;
        }

        float best = std::numeric_limits<float>::infinity();
        glm::vec3 best_a(0.0f);
        glm::vec3 best_b(0.0f);

        for (size_t i = 0; i + 2 < triangles_a.size(); i += 3) {
            for (size_t j = 0; j + 2 < triangles_b.size(); j += 3) {
                glm::vec3 point_a(0.0f);
                glm::vec3 point_b(0.0f);
                const float distance = TriangleTriangleDistance(triangles_a[i], triangles_a[i + 1], triangles_a[i + 2],
                                                                triangles_b[j], triangles_b[j + 1], triangles_b[j + 2], &point_a,
                                                                &point_b);
                if (distance < best) {
                    best   = distance;
                    best_a = point_a;
                    best_b = point_b;
                }
            }
        }

        result.distance_m = best;
        result.point_a    = best_a;
        result.point_b    = best_b;
        return result;
    }

    CollisionPairDistance BuildCollisionPairDistanceAabb(const rkv::RobotScene::LinkCollisionProxy& a,
                                                         const rkv::RobotScene::LinkCollisionProxy& b) {
        CollisionPairDistance result;
        result.link_a            = a.link_name;
        result.link_b            = b.link_name;
        result.center_distance_m = glm::length(b.world_center - a.world_center);

        if (a.has_world_aabb && b.has_world_aabb) {
            result.surface_distance_m = DistanceAabbToAabb(a.world_aabb_min, a.world_aabb_max, b.world_aabb_min, b.world_aabb_max,
                                                           &result.point_a, &result.point_b);
            return result;
        }

        const glm::vec3 delta       = b.world_center - a.world_center;
        const float center_distance = glm::length(delta);
        result.center_distance_m    = center_distance;

        glm::vec3 direction(1.0f, 0.0f, 0.0f);
        if (center_distance > kDistanceEpsilon) {
            direction = delta / center_distance;
        }

        result.point_a            = a.world_center + direction * a.radius_m;
        result.point_b            = b.world_center - direction * b.radius_m;
        result.surface_distance_m = center_distance - (a.radius_m + b.radius_m);
        return result;
    }

    MeshDistanceResult MeshDistanceBetweenLinks(const rkv::RobotScene& scene, const std::string& link_a,
                                                const std::string& link_b) {
        std::vector<glm::vec3> triangles_a;
        std::vector<glm::vec3> triangles_b;
        scene.appendLinkWorldCollisionTriangles(link_a, &triangles_a);
        scene.appendLinkWorldCollisionTriangles(link_b, &triangles_b);
        return MinDistanceBetweenTriangleSoups(triangles_a, triangles_b);
    }

    CollisionPairDistance RefineCollisionPairDistanceWithMesh(const rkv::RobotScene& scene,
                                                              const CollisionPairDistance& aabb_distance) {
        CollisionPairDistance result     = aabb_distance;
        const MeshDistanceResult mesh_distance = MeshDistanceBetweenLinks(scene, aabb_distance.link_a, aabb_distance.link_b);
        if (!std::isfinite(mesh_distance.distance_m)) {
            return result;
        }
        result.surface_distance_m = mesh_distance.distance_m;
        result.point_a            = mesh_distance.point_a;
        result.point_b            = mesh_distance.point_b;
        return result;
    }

    CollisionPairDistance BuildCollisionPairDistance(const rkv::RobotScene::LinkCollisionProxy& a,
                                                     const rkv::RobotScene::LinkCollisionProxy& b,
                                                     const rkv::RobotScene* scene_for_mesh, bool use_mesh) {
        CollisionPairDistance result = BuildCollisionPairDistanceAabb(a, b);
        if (use_mesh && scene_for_mesh != nullptr) {
            return RefineCollisionPairDistanceWithMesh(*scene_for_mesh, result);
        }
        return result;
    }

}  // namespace kinematic_viewer
