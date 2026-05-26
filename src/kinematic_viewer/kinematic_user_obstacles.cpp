#include "kinematic_viewer/kinematic_user_obstacles.h"
#include "kinematic_viewer/kinematic_angle_units.h"
#include "kinematic_viewer/kinematic_string_utils.h"

#include "kinematic_viewer/kinematic_marker_utils.h"

#include "imgui.h"
#include <yaml-cpp/yaml.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <filesystem>
#include <string>
#include <vector>

namespace kinematic_viewer {
    namespace {

        using kinematic_viewer::FormatPoseInputXyzQuat;
        using kinematic_viewer::NormalizePath;
        using kinematic_viewer::ParsePoseInputXyzQuat;

        constexpr int kSphereStacks   = 16;
        constexpr int kSphereSectors  = 24;
        constexpr int kCylinderSlices = 28;

        struct VertexPN {
            glm::vec3 p;
            glm::vec3 n;
            glm::vec2 uv{0.0f};
        };

        bool IsYamlFileExt(const std::filesystem::path& path) {
            std::string ext = path.extension().string();
            std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
            return ext == ".yaml" || ext == ".yml";
        }

        const char* ObstacleKindName(UserObstacleItem::Kind kind) {
            switch (kind) {
                case UserObstacleItem::Kind::Box:
                    return "box";
                case UserObstacleItem::Kind::Sphere:
                    return "sphere";
                case UserObstacleItem::Kind::Cylinder:
                    return "cylinder";
                default:
                    return "unknown";
            }
        }

        UserObstacleItem::Kind ObstacleKindFromName(const std::string& name) {
            std::string key = name;
            std::transform(key.begin(), key.end(), key.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
            if (key == "sphere") {
                return UserObstacleItem::Kind::Sphere;
            }
            if (key == "cylinder") {
                return UserObstacleItem::Kind::Cylinder;
            }
            return UserObstacleItem::Kind::Box;
        }

        void pushMeshToGpu(const std::vector<VertexPN>& verts, const std::vector<unsigned int>& indices, GLuint* vao, GLuint* vbo,
                           GLuint* ebo, GLsizei* out_index_count) {
            *vao             = 0;
            *vbo             = 0;
            *ebo             = 0;
            *out_index_count = static_cast<GLsizei>(indices.size());
            if (verts.empty() || indices.empty()) {
                return;
            }

            std::vector<float> interleaved;
            interleaved.reserve(verts.size() * 8);
            for (const auto& v : verts) {
                interleaved.push_back(v.p.x);
                interleaved.push_back(v.p.y);
                interleaved.push_back(v.p.z);
                interleaved.push_back(v.n.x);
                interleaved.push_back(v.n.y);
                interleaved.push_back(v.n.z);
                interleaved.push_back(v.uv.x);
                interleaved.push_back(v.uv.y);
            }

            glGenVertexArrays(1, vao);
            glGenBuffers(1, vbo);
            glGenBuffers(1, ebo);
            glBindVertexArray(*vao);
            glBindBuffer(GL_ARRAY_BUFFER, *vbo);
            glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(interleaved.size() * sizeof(float)), interleaved.data(), GL_STATIC_DRAW);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, *ebo);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(indices.size() * sizeof(unsigned int)), indices.data(),
                         GL_STATIC_DRAW);
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 8, reinterpret_cast<void*>(0));
            glEnableVertexAttribArray(1);
            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 8, reinterpret_cast<void*>(sizeof(float) * 3));
            glEnableVertexAttribArray(2);
            glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 8, reinterpret_cast<void*>(sizeof(float) * 6));
            glBindVertexArray(0);
        }

        void destroyMeshGpu(GLuint* vao, GLuint* vbo, GLuint* ebo) {
            if (*ebo) {
                glDeleteBuffers(1, ebo);
                *ebo = 0;
            }
            if (*vbo) {
                glDeleteBuffers(1, vbo);
                *vbo = 0;
            }
            if (*vao) {
                glDeleteVertexArrays(1, vao);
                *vao = 0;
            }
        }

        std::vector<VertexPN> buildUnitCube() {
            const glm::vec3 half(0.5f);
            const glm::vec3 corners[8] = {
                {-half.x, -half.y, -half.z}, {half.x, -half.y, -half.z}, {half.x, half.y, -half.z}, {-half.x, half.y, -half.z},
                {-half.x, -half.y, half.z},  {half.x, -half.y, half.z},  {half.x, half.y, half.z},  {-half.x, half.y, half.z},
            };
            const int faces[6][4] = {
                {0, 1, 2, 3},  // -Z
                {4, 5, 6, 7},  // +Z
                {0, 4, 7, 3},  // -X
                {1, 5, 6, 2},  // +X
                {0, 1, 5, 4},  // -Y
                {3, 2, 6, 7},  // +Y
            };
            const glm::vec3 normals[6] = {
                {0.0f, 0.0f, -1.0f}, {0.0f, 0.0f, 1.0f}, {-1.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, -1.0f, 0.0f}, {0.0f, 1.0f, 0.0f},
            };
            std::vector<VertexPN> out;
            for (int f = 0; f < 6; ++f) {
                const glm::vec3& n = normals[f];
                const int* c       = faces[f];
                glm::vec3 v0       = corners[c[0]];
                glm::vec3 v1       = corners[c[1]];
                glm::vec3 v2       = corners[c[2]];
                glm::vec3 v3       = corners[c[3]];
                unsigned base      = static_cast<unsigned>(out.size());
                out.push_back({v0, n});
                out.push_back({v1, n});
                out.push_back({v2, n});
                out.push_back({v3, n});
            }
            return out;
        }

        std::pair<std::vector<VertexPN>, std::vector<unsigned int>> buildUvSphere() {
            std::vector<VertexPN> verts;
            for (int iy = 0; iy <= kSphereStacks; ++iy) {
                const float stack_angle = glm::pi<float>() * static_cast<float>(iy) / static_cast<float>(kSphereStacks);
                const float xy          = 0.5f * std::sin(stack_angle);
                const float z           = 0.5f * std::cos(stack_angle);
                for (int ix = 0; ix <= kSphereSectors; ++ix) {
                    const float sector_angle = glm::two_pi<float>() * static_cast<float>(ix) / static_cast<float>(kSphereSectors);
                    const float x            = xy * std::cos(sector_angle);
                    const float y            = xy * std::sin(sector_angle);
                    const glm::vec3 p(x, y, z);
                    glm::vec3 n = p * 2.0f;
                    if (glm::length(n) > 1e-8f) {
                        n = glm::normalize(n);
                    }
                    const float u = static_cast<float>(ix) / static_cast<float>(kSphereSectors);
                    const float v = static_cast<float>(iy) / static_cast<float>(kSphereStacks);
                    verts.push_back({p, n, {u, v}});
                }
            }
            std::vector<unsigned int> indices;
            for (int iy = 0; iy < kSphereStacks; ++iy) {
                for (int ix = 0; ix < kSphereSectors; ++ix) {
                    const int cur  = iy * (kSphereSectors + 1) + ix;
                    const int next = cur + kSphereSectors + 1;
                    indices.push_back(static_cast<unsigned>(cur));
                    indices.push_back(static_cast<unsigned>(next));
                    indices.push_back(static_cast<unsigned>(cur + 1));
                    indices.push_back(static_cast<unsigned>(cur + 1));
                    indices.push_back(static_cast<unsigned>(next));
                    indices.push_back(static_cast<unsigned>(next + 1));
                }
            }
            return {verts, indices};
        }

        std::pair<std::vector<VertexPN>, std::vector<unsigned int>> buildUnitCylinder() {
            std::vector<VertexPN> verts;
            std::vector<unsigned int> indices;
            const float y0 = -0.5f;
            const float y1 = 0.5f;
            const float r  = 0.5f;

            const unsigned bottom_center = static_cast<unsigned>(verts.size());
            verts.push_back({{0.0f, y0, 0.0f}, {0.0f, -1.0f, 0.0f}, {0.5f, 0.5f}});
            for (int i = 0; i <= kCylinderSlices; ++i) {
                const float t = glm::two_pi<float>() * static_cast<float>(i) / static_cast<float>(kCylinderSlices);
                const float x = r * std::cos(t);
                const float z = r * std::sin(t);
                const glm::vec3 n(0.0f, -1.0f, 0.0f);
                verts.push_back({{x, y0, z}, n, {static_cast<float>(i) / static_cast<float>(kCylinderSlices), 0.0f}});
            }
            for (int i = 0; i < kCylinderSlices; ++i) {
                indices.push_back(bottom_center);
                indices.push_back(bottom_center + 1 + static_cast<unsigned>(i));
                indices.push_back(bottom_center + 2 + static_cast<unsigned>(i));
            }

            const unsigned top_center = static_cast<unsigned>(verts.size());
            verts.push_back({{0.0f, y1, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.5f, 0.5f}});
            for (int i = 0; i <= kCylinderSlices; ++i) {
                const float t = glm::two_pi<float>() * static_cast<float>(i) / static_cast<float>(kCylinderSlices);
                const float x = r * std::cos(t);
                const float z = r * std::sin(t);
                const glm::vec3 n(0.0f, 1.0f, 0.0f);
                verts.push_back({{x, y1, z}, n, {static_cast<float>(i) / static_cast<float>(kCylinderSlices), 1.0f}});
            }
            for (int i = 0; i < kCylinderSlices; ++i) {
                indices.push_back(top_center);
                indices.push_back(top_center + 2 + static_cast<unsigned>(i));
                indices.push_back(top_center + 1 + static_cast<unsigned>(i));
            }

            const unsigned side_base = static_cast<unsigned>(verts.size());
            for (int i = 0; i <= kCylinderSlices; ++i) {
                const float t = glm::two_pi<float>() * static_cast<float>(i) / static_cast<float>(kCylinderSlices);
                const float x = std::cos(t);
                const float z = std::sin(t);
                glm::vec3 n(x, 0.0f, z);
                n = glm::normalize(n);
                verts.push_back({{r * x, y0, r * z}, n, {static_cast<float>(i) / static_cast<float>(kCylinderSlices), 0.0f}});
                verts.push_back({{r * x, y1, r * z}, n, {static_cast<float>(i) / static_cast<float>(kCylinderSlices), 1.0f}});
            }
            for (int i = 0; i < kCylinderSlices; ++i) {
                const unsigned b = side_base + static_cast<unsigned>(i) * 2;
                indices.push_back(b + 0);
                indices.push_back(b + 1);
                indices.push_back(b + 2);
                indices.push_back(b + 1);
                indices.push_back(b + 3);
                indices.push_back(b + 2);
            }

            return {verts, indices};
        }

        float sdBox(glm::vec3 p, glm::vec3 half_extents) {
            glm::vec3 q = glm::abs(p) - half_extents;
            return glm::length(glm::max(q, glm::vec3(0.0f))) + std::min(std::max(q.x, std::max(q.y, q.z)), 0.0f);
        }

        glm::vec3 closestPointOnObbSurfaceWorld(const glm::vec3& world_point, const glm::mat4& world_from_local,
                                                const glm::vec3& half_extents) {
            const glm::mat4 local_from_world = glm::inverse(world_from_local);
            const glm::vec3 local            = glm::vec3(local_from_world * glm::vec4(world_point, 1.0f));
            const glm::vec3 clamped          = glm::clamp(local, -half_extents, half_extents);
            const glm::vec3 delta            = local - clamped;
            if (glm::length(delta) < 1e-8f) {
                glm::vec3 ax(1.0f, 0.0f, 0.0f);
                float best = 1e9f;
                glm::vec3 best_p(0.0f);
                for (int axis = 0; axis < 3; ++axis) {
                    for (float s : {-1.0f, 1.0f}) {
                        glm::vec3 ptry  = local;
                        ptry[axis]      = s * half_extents[axis];
                        const float pen = glm::length(glm::max(glm::abs(ptry) - half_extents, glm::vec3(0.0f)));
                        if (pen < best) {
                            best   = pen;
                            best_p = ptry;
                        }
                    }
                }
                return glm::vec3(world_from_local * glm::vec4(best_p, 1.0f));
            }
            return glm::vec3(world_from_local * glm::vec4(clamped, 1.0f));
        }

        void evalLinkVsObstacle(const teleop_viewer::RobotScene::LinkCollisionProxy& link, const UserObstacleItem& obs,
                                float* surface_distance_m, float* center_distance_m, glm::vec3* point_on_link,
                                glm::vec3* point_on_obstacle) {
            const glm::vec3 cL = link.world_center;
            const float rL     = std::max(1e-5f, link.radius_m);

            if (obs.kind == UserObstacleItem::Kind::Sphere) {
                const float rO = std::max(1e-5f, obs.params.x);
                const glm::vec3 cO(obs.position);
                const glm::vec3 d   = cO - cL;
                const float dc      = glm::length(d);
                const glm::vec3 dir = dc > 1e-8f ? d / dc : glm::vec3(1.0f, 0.0f, 0.0f);
                *center_distance_m  = dc;
                *surface_distance_m = dc - (rL + rO);
                *point_on_link      = cL + dir * rL;
                *point_on_obstacle  = cO - dir * rO;
                return;
            }

            if (obs.kind == UserObstacleItem::Kind::Box) {
                const glm::mat4 M = markerWorldMatrix(obs.position, obs.rpy_deg);
                const glm::vec3 he(0.5f * std::max(1e-4f, obs.params.x), 0.5f * std::max(1e-4f, obs.params.y),
                                   0.5f * std::max(1e-4f, obs.params.z));
                const glm::mat4 invM   = glm::inverse(M);
                const glm::vec3 local  = glm::vec3(invM * glm::vec4(cL, 1.0f));
                const float sdf        = sdBox(local, he);
                *surface_distance_m    = sdf - rL;
                *center_distance_m     = std::max(0.0f, sdf) + rL;
                const glm::vec3 on_obs = closestPointOnObbSurfaceWorld(cL, M, he);
                glm::vec3 dir2         = on_obs - cL;
                const float l2         = glm::length(dir2);
                if (l2 > 1e-8f) {
                    dir2 /= l2;
                } else {
                    dir2 = glm::vec3(1.0f, 0.0f, 0.0f);
                }
                *point_on_link     = cL + dir2 * rL;
                *point_on_obstacle = on_obs;
                return;
            }

            const float cyl_r        = std::max(1e-4f, obs.params.x);
            const float cyl_h        = std::max(1e-4f, obs.params.y);
            const glm::mat4 rot_only = markerWorldMatrix(glm::vec3(0.0f), obs.rpy_deg);
            const glm::vec3 axis_y   = glm::normalize(glm::vec3(rot_only * glm::vec4(0.0f, 1.0f, 0.0f, 0.0f)));
            const glm::vec3 center   = obs.position;
            const float hh           = 0.5f * cyl_h;
            const glm::vec3 cap_a    = center - axis_y * hh;
            const glm::vec3 cap_b    = center + axis_y * hh;
            const glm::vec3 ab       = cap_b - cap_a;
            const float ab_len2      = glm::dot(ab, ab);
            const float t            = ab_len2 > 1e-12f ? glm::dot(cL - cap_a, ab) / ab_len2 : 0.0f;
            const float t_clamped    = std::clamp(t, 0.0f, 1.0f);
            const glm::vec3 q        = cap_a + ab * t_clamped;
            const glm::vec3 radial   = cL - q;
            const float rad_len      = glm::length(radial);
            glm::vec3 on_side        = q;
            if (rad_len > 1e-8f) {
                on_side = q + (radial / rad_len) * cyl_r;
            } else {
                const glm::vec3 px = glm::normalize(glm::vec3(rot_only * glm::vec4(1.0f, 0.0f, 0.0f, 0.0f)));
                on_side            = q + px * cyl_r;
            }
            const float bound_r = std::sqrt(cyl_r * cyl_r + hh * hh);
            const float dc      = glm::length(cL - center);
            *center_distance_m  = dc;
            *surface_distance_m = dc - (rL + bound_r);
            glm::vec3 dir3      = on_side - cL;
            const float l3      = glm::length(dir3);
            if (l3 > 1e-8f) {
                dir3 /= l3;
            } else {
                dir3 = glm::vec3(1.0f, 0.0f, 0.0f);
            }
            *point_on_link     = cL + dir3 * rL;
            *point_on_obstacle = on_side;
        }

    }  // namespace

    bool InitUserObstacleGpuMeshes(UserObstacleGpuMeshes* gpu) {
        if (gpu == nullptr) {
            return false;
        }
        *gpu = UserObstacleGpuMeshes{};

        {
            std::vector<VertexPN> cube_verts = buildUnitCube();
            std::vector<unsigned int> cube_ix;
            cube_ix.reserve(36);
            for (unsigned i = 0; i < 24; i += 4) {
                cube_ix.push_back(i + 0);
                cube_ix.push_back(i + 1);
                cube_ix.push_back(i + 2);
                cube_ix.push_back(i + 0);
                cube_ix.push_back(i + 2);
                cube_ix.push_back(i + 3);
            }
            pushMeshToGpu(cube_verts, cube_ix, &gpu->box_vao, &gpu->box_vbo, &gpu->box_ebo, &gpu->box_index_count);
        }

        {
            auto sp = buildUvSphere();
            pushMeshToGpu(sp.first, sp.second, &gpu->sphere_vao, &gpu->sphere_vbo, &gpu->sphere_ebo, &gpu->sphere_index_count);
        }

        {
            auto cy = buildUnitCylinder();
            pushMeshToGpu(cy.first, cy.second, &gpu->cylinder_vao, &gpu->cylinder_vbo, &gpu->cylinder_ebo, &gpu->cylinder_index_count);
        }

        return gpu->box_vao && gpu->sphere_vao && gpu->cylinder_vao;
    }

    void DestroyUserObstacleGpuMeshes(UserObstacleGpuMeshes* gpu) {
        if (gpu == nullptr) {
            return;
        }
        destroyMeshGpu(&gpu->box_vao, &gpu->box_vbo, &gpu->box_ebo);
        destroyMeshGpu(&gpu->sphere_vao, &gpu->sphere_vbo, &gpu->sphere_ebo);
        destroyMeshGpu(&gpu->cylinder_vao, &gpu->cylinder_vbo, &gpu->cylinder_ebo);
        gpu->box_index_count = gpu->sphere_index_count = gpu->cylinder_index_count = 0;
    }

    void DrawUserObstacles(GLuint mesh_shader, const UserObstacleState& obstacles, const UserObstacleGpuMeshes& gpu, const glm::mat4& view,
                           const glm::mat4& projection) {
        glUseProgram(mesh_shader);
        glUniformMatrix4fv(glGetUniformLocation(mesh_shader, "view"), 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(glGetUniformLocation(mesh_shader, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
        glUniform1i(glGetUniformLocation(mesh_shader, "hasTexture"), false);
        const GLint loc_alpha = glGetUniformLocation(mesh_shader, "materialAlpha");
        if (loc_alpha >= 0) {
            glUniform1f(loc_alpha, 1.0f);
        }

        for (const auto& o : obstacles.items) {
            if (!o.visible) {
                continue;
            }

            glm::mat4 model = markerWorldMatrix(o.position, o.rpy_deg);
            GLuint vao      = 0;
            GLsizei icount  = 0;
            if (o.kind == UserObstacleItem::Kind::Box) {
                model  = model * glm::scale(glm::mat4(1.0f), glm::vec3(std::max(1e-4f, o.params.x), std::max(1e-4f, o.params.y),
                                                                       std::max(1e-4f, o.params.z)));
                vao    = gpu.box_vao;
                icount = gpu.box_index_count;
            } else if (o.kind == UserObstacleItem::Kind::Sphere) {
                const float d = 2.0f * std::max(1e-4f, o.params.x);
                model         = model * glm::scale(glm::mat4(1.0f), glm::vec3(d));
                vao           = gpu.sphere_vao;
                icount        = gpu.sphere_index_count;
            } else {
                const float r = std::max(1e-4f, o.params.x);
                const float h = std::max(1e-4f, o.params.y);
                model         = model * glm::scale(glm::mat4(1.0f), glm::vec3(2.0f * r, h, 2.0f * r));
                vao           = gpu.cylinder_vao;
                icount        = gpu.cylinder_index_count;
            }

            if (!vao || icount <= 0) {
                continue;
            }

            glUniformMatrix4fv(glGetUniformLocation(mesh_shader, "model"), 1, GL_FALSE, glm::value_ptr(model));
            glUniform3f(glGetUniformLocation(mesh_shader, "diffuseColor"), o.color.r, o.color.g, o.color.b);
            glBindVertexArray(vao);
            glDrawElements(GL_TRIANGLES, icount, GL_UNSIGNED_INT, nullptr);
            glBindVertexArray(0);
        }
    }

    void RenderUserObstaclePanel(UserObstacleState* st, bool angle_unit_deg) {
        if (st == nullptr) {
            return;
        }
        const auto draw_rpy_drag = [&](const char* label, glm::vec3* rpy_deg_stored) {
            if (rpy_deg_stored == nullptr) {
                return;
            }
            glm::vec3 rpy_ui = RpyUiFromDegStored(*rpy_deg_stored, angle_unit_deg);
            if (ImGui::DragFloat3(label, glm::value_ptr(rpy_ui), angle_unit_deg ? 0.2f : 0.01f, AngleDragMin(angle_unit_deg),
                                  AngleDragMax(angle_unit_deg), AngleInputFormat(angle_unit_deg))) {
                *rpy_deg_stored = RpyUiToDegStored(rpy_ui, angle_unit_deg);
            }
        };
        ImGui::TextUnformatted("自定义障碍物");
        static std::vector<UserObstacleItem> undo_items;
        static int undo_selected_index = -1;
        static bool undo_valid         = false;
        static std::string undo_label;
        auto capture_undo = [&](const char* label) {
            undo_items          = st->items;
            undo_selected_index = st->selected_index;
            undo_valid          = true;
            undo_label          = label ? label : "";
        };
        auto try_update_next_serial = [&]() {
            int max_serial = 0;
            for (const auto& it : st->items) {
                int value = 0;
                if (std::sscanf(it.name.c_str(), "%*[^0-9]%d", &value) == 1) {
                    max_serial = std::max(max_serial, value);
                }
            }
            st->next_serial = std::max(st->next_serial, max_serial + 1);
        };
        if (ImGui::Button("撤销上一步") && undo_valid) {
            st->items          = undo_items;
            st->selected_index = undo_selected_index;
            try_update_next_serial();
            undo_valid = false;
        }
        if (undo_valid) {
            ImGui::SameLine();
            ImGui::TextDisabled("可撤销: %s", undo_label.c_str());
        }
        ImGui::Separator();
        ImGui::Checkbox("参与安全距离计算", &st->affect_collision);
        ImGui::SameLine();
        ImGui::Checkbox("3D Gizmo编辑", &st->enable_pose_gizmo);
        if (st->enable_pose_gizmo) {
            ImGui::TextUnformatted("Gizmo");
            ImGui::SameLine();
            ImGui::RadioButton("平移##obs", &st->gizmo_operation, 0);
            ImGui::SameLine();
            ImGui::RadioButton("旋转##obs", &st->gizmo_operation, 1);
            ImGui::SameLine();
            ImGui::RadioButton("平移+旋转##obs", &st->gizmo_operation, 2);
            ImGui::SameLine();
            ImGui::TextUnformatted("| 坐标系");
            ImGui::SameLine();
            if (ImGui::RadioButton("WORLD##obs", st->gizmo_world_mode)) {
                st->gizmo_world_mode = true;
            }
            ImGui::SameLine();
            if (ImGui::RadioButton("LOCAL##obs", !st->gizmo_world_mode)) {
                st->gizmo_world_mode = false;
            }
            ImGui::SliderFloat("Gizmo 尺寸##obs", &st->gizmo_size_clip_space, 0.08f, 0.35f, "%.2f");
            ImGui::TextDisabled("可在3D视图里直接拖动选中障碍物");
        }
        ImGui::TextDisabled("盒子/球体为解析近似；圆柱使用外接球保守估计。");
        ImGui::Separator();
        if (ImGui::CollapsingHeader("文件 I/O（导入/导出）", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::TextUnformatted("导出障碍物");
            static char export_dir[384]         = "config";
            static char export_name[128]        = "user_obstacles";
            static char export_path_full[512]   = "";
            static char export_browser_dir[512] = "";
            static std::string export_status;
            ImGui::InputText("导出目录(可选)", export_dir, sizeof(export_dir));
            ImGui::InputText("导出文件名(可自定义)", export_name, sizeof(export_name));
            ImGui::InputText("或直接完整路径", export_path_full, sizeof(export_path_full));
            if (ImGui::Button("浏览本地路径")) {
                const std::string default_dir = NormalizePath(std::filesystem::current_path().string());
                std::snprintf(export_browser_dir, sizeof(export_browser_dir), "%s", default_dir.c_str());
                ImGui::OpenPopup("obstacle_export_path_browser_popup");
            }
            if (ImGui::BeginPopupModal("obstacle_export_path_browser_popup", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
                ImGui::InputText("浏览目录", export_browser_dir, sizeof(export_browser_dir));
                ImGui::SameLine();
                if (ImGui::Button("进入目录##obs_export")) {
                    const std::string normalized = NormalizePath(export_browser_dir);
                    std::snprintf(export_browser_dir, sizeof(export_browser_dir), "%s", normalized.c_str());
                }
                ImGui::SameLine();
                if (ImGui::Button("上一级##obs_export")) {
                    std::filesystem::path current = std::filesystem::path(NormalizePath(export_browser_dir));
                    std::filesystem::path parent  = current.parent_path();
                    if (parent.empty()) {
                        parent = std::filesystem::path("/");
                    }
                    std::snprintf(export_browser_dir, sizeof(export_browser_dir), "%s", parent.string().c_str());
                }
                ImGui::SameLine();
                if (ImGui::Button("HOME##obs_export")) {
                    const char* home = std::getenv("HOME");
                    if (home != nullptr && home[0] != '\0') {
                        std::snprintf(export_browser_dir, sizeof(export_browser_dir), "%s", home);
                    }
                }

                std::error_code ec;
                const std::filesystem::path browse_path(export_browser_dir);
                if (!std::filesystem::exists(browse_path, ec) || !std::filesystem::is_directory(browse_path, ec)) {
                    ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.45f, 1.0f), "目录不可用");
                } else {
                    static char new_file_name[128] = "new_obstacles.yaml";
                    std::vector<std::filesystem::path> dirs;
                    std::vector<std::filesystem::path> files;
                    for (auto it = std::filesystem::directory_iterator(browse_path, ec); !ec && it != std::filesystem::directory_iterator();
                         ++it) {
                        if (it->is_directory(ec)) {
                            dirs.push_back(it->path());
                        } else if (it->is_regular_file(ec) && IsYamlFileExt(it->path())) {
                            files.push_back(it->path());
                        }
                    }
                    std::sort(dirs.begin(), dirs.end());
                    std::sort(files.begin(), files.end());

                    if (ImGui::BeginChild("obstacle_export_path_browser_list", ImVec2(620, 280), true)) {
                        if (ImGui::BeginPopupContextWindow("obstacle_export_context_menu", ImGuiPopupFlags_MouseButtonRight)) {
                            ImGui::TextUnformatted("新建文件到当前目录");
                            ImGui::InputText("文件名", new_file_name, sizeof(new_file_name));
                            if (ImGui::Button("创建文件")) {
                                std::string filename = new_file_name;
                                if (filename.empty()) {
                                    export_status = "创建失败: 文件名为空";
                                } else {
                                    if (filename.size() < 5 || filename.substr(filename.size() - 5) != ".yaml") {
                                        filename += ".yaml";
                                    }
                                    std::filesystem::path out_file = browse_path / filename;
                                    if (std::filesystem::exists(out_file)) {
                                        export_status = std::string("创建失败: 文件已存在 ") + out_file.string();
                                    } else {
                                        std::ofstream create_ofs(out_file, std::ios::out);
                                        if (!create_ofs.is_open()) {
                                            export_status = std::string("创建失败: 无法创建文件 ") + out_file.string();
                                        } else {
                                            create_ofs << "obstacles:\n";
                                            if (create_ofs.good()) {
                                                export_status = std::string("创建成功: ") + out_file.string();
                                                std::snprintf(export_path_full, sizeof(export_path_full), "%s", out_file.string().c_str());
                                            } else {
                                                export_status = std::string("创建失败: 写入文件时出错 ") + out_file.string();
                                            }
                                        }
                                    }
                                }
                                ImGui::CloseCurrentPopup();
                            }
                            ImGui::SameLine();
                            if (ImGui::Button("取消##create_file_cancel")) {
                                ImGui::CloseCurrentPopup();
                            }
                            ImGui::EndPopup();
                        }
                        for (const auto& d : dirs) {
                            std::string label = "[DIR] " + d.filename().string();
                            if (ImGui::Selectable(label.c_str(), false)) {
                                std::snprintf(export_browser_dir, sizeof(export_browser_dir), "%s", d.string().c_str());
                            }
                        }
                        for (const auto& f : files) {
                            std::string label = f.filename().string();
                            if (ImGui::Selectable(label.c_str(), false)) {
                                std::snprintf(export_path_full, sizeof(export_path_full), "%s", f.string().c_str());
                            }
                        }
                        ImGui::EndChild();
                    }
                }

                if (ImGui::Button("使用当前目录+文件名")) {
                    std::string filename = export_name;
                    if (filename.empty()) {
                        filename = "user_obstacles";
                    }
                    if (filename.size() < 5 || filename.substr(filename.size() - 5) != ".yaml") {
                        filename += ".yaml";
                    }
                    std::filesystem::path out    = std::filesystem::path(export_browser_dir) / filename;
                    const std::string final_path = out.string();
                    std::snprintf(export_path_full, sizeof(export_path_full), "%s", final_path.c_str());
                    ImGui::CloseCurrentPopup();
                }
                ImGui::SameLine();
                if (ImGui::Button("关闭##obs_export_browser")) {
                    ImGui::CloseCurrentPopup();
                }
                ImGui::EndPopup();
            }
            ImGui::TextDisabled("若填写完整路径则优先使用；文件名自动补 .yaml");
            if (ImGui::Button("导出已添加障碍物到文件")) {
                std::string final_path;
                if (export_path_full[0] != '\0') {
                    final_path = export_path_full;
                } else {
                    std::string filename = export_name;
                    if (filename.empty()) {
                        filename = "user_obstacles";
                    }
                    if (filename.size() < 5 || filename.substr(filename.size() - 5) != ".yaml") {
                        filename += ".yaml";
                    }
                    std::filesystem::path out =
                        (export_dir[0] == '\0') ? std::filesystem::path(filename) : (std::filesystem::path(export_dir) / filename);
                    final_path = out.string();
                }
                std::ofstream ofs(final_path, std::ios::out | std::ios::trunc);
                if (!ofs.is_open()) {
                    export_status = std::string("导出失败: 无法打开文件 ") + final_path;
                } else {
                    ofs << "obstacles:\n";
                    for (const auto& o : st->items) {
                        ofs << "  - name: \"" << o.name << "\"\n";
                        ofs << "    kind: " << ObstacleKindName(o.kind) << "\n";
                        ofs << "    visible: " << (o.visible ? "true" : "false") << "\n";
                        ofs << "    color: [" << o.color.x << ", " << o.color.y << ", " << o.color.z << "]\n";
                        ofs << "    position: [" << o.position.x << ", " << o.position.y << ", " << o.position.z << "]\n";
                        ofs << "    rpy_deg: [" << o.rpy_deg.x << ", " << o.rpy_deg.y << ", " << o.rpy_deg.z << "]\n";
                        ofs << "    params: [" << o.params.x << ", " << o.params.y << ", " << o.params.z << "]\n";
                    }
                    if (ofs.good()) {
                        export_status = std::string("导出成功: ") + final_path + " (共 " + std::to_string(st->items.size()) + " 个)";
                        std::snprintf(export_path_full, sizeof(export_path_full), "%s", final_path.c_str());
                    } else {
                        export_status = std::string("导出失败: 写入文件时出错 ") + final_path;
                    }
                }
            }
            if (!export_status.empty()) {
                const bool ok = export_status.find("成功") != std::string::npos;
                ImGui::TextColored(ok ? ImVec4(0.60f, 0.92f, 0.60f, 1.0f) : ImVec4(1.0f, 0.45f, 0.45f, 1.0f), "%s", export_status.c_str());
            }

            ImGui::Separator();
            ImGui::TextUnformatted("导入障碍物");
            static char import_path[512]        = "config/user_obstacles.yaml";
            static char import_browser_dir[512] = "";
            static std::string import_status;
            ImGui::InputText("导入文件路径", import_path, sizeof(import_path));
            if (ImGui::Button("浏览导入文件")) {
                const std::string default_dir = NormalizePath(std::filesystem::current_path().string());
                std::snprintf(import_browser_dir, sizeof(import_browser_dir), "%s", default_dir.c_str());
                ImGui::OpenPopup("obstacle_import_file_browser_popup");
            }
            if (ImGui::BeginPopupModal("obstacle_import_file_browser_popup", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
                ImGui::InputText("目录", import_browser_dir, sizeof(import_browser_dir));
                ImGui::SameLine();
                if (ImGui::Button("进入目录##obs_import")) {
                    const std::string normalized = NormalizePath(import_browser_dir);
                    std::snprintf(import_browser_dir, sizeof(import_browser_dir), "%s", normalized.c_str());
                }
                ImGui::SameLine();
                if (ImGui::Button("上一级##obs_import")) {
                    std::filesystem::path current = std::filesystem::path(NormalizePath(import_browser_dir));
                    std::filesystem::path parent  = current.parent_path();
                    if (parent.empty()) {
                        parent = std::filesystem::path("/");
                    }
                    std::snprintf(import_browser_dir, sizeof(import_browser_dir), "%s", parent.string().c_str());
                }
                ImGui::SameLine();
                if (ImGui::Button("HOME##obs_import")) {
                    const char* home = std::getenv("HOME");
                    if (home != nullptr && home[0] != '\0') {
                        std::snprintf(import_browser_dir, sizeof(import_browser_dir), "%s", home);
                    }
                }

                std::error_code ec;
                const std::filesystem::path browse_path(import_browser_dir);
                if (!std::filesystem::exists(browse_path, ec) || !std::filesystem::is_directory(browse_path, ec)) {
                    ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.45f, 1.0f), "目录不可用");
                } else {
                    std::vector<std::filesystem::path> dirs;
                    std::vector<std::filesystem::path> files;
                    for (auto it = std::filesystem::directory_iterator(browse_path, ec); !ec && it != std::filesystem::directory_iterator();
                         ++it) {
                        if (it->is_directory(ec)) {
                            dirs.push_back(it->path());
                        } else if (it->is_regular_file(ec) && IsYamlFileExt(it->path())) {
                            files.push_back(it->path());
                        }
                    }
                    std::sort(dirs.begin(), dirs.end());
                    std::sort(files.begin(), files.end());
                    if (ImGui::BeginChild("obstacle_import_file_browser_list", ImVec2(620, 280), true)) {
                        for (const auto& d : dirs) {
                            std::string label = "[DIR] " + d.filename().string();
                            if (ImGui::Selectable(label.c_str(), false)) {
                                std::snprintf(import_browser_dir, sizeof(import_browser_dir), "%s", d.string().c_str());
                            }
                        }
                        for (const auto& f : files) {
                            std::string label = f.filename().string();
                            if (ImGui::Selectable(label.c_str(), false)) {
                                std::snprintf(import_path, sizeof(import_path), "%s", f.string().c_str());
                                ImGui::CloseCurrentPopup();
                            }
                        }
                        ImGui::EndChild();
                    }
                }
                if (ImGui::Button("关闭##obs_import_browser")) {
                    ImGui::CloseCurrentPopup();
                }
                ImGui::EndPopup();
            }
            if (ImGui::Button("导入并替换当前障碍物")) {
                try {
                    YAML::Node root      = YAML::LoadFile(import_path);
                    YAML::Node obs_nodes = root["obstacles"];
                    if (!obs_nodes || !obs_nodes.IsSequence()) {
                        import_status = "导入失败: YAML 缺少 obstacles 数组";
                    } else {
                        capture_undo("导入替换");
                        std::vector<UserObstacleItem> loaded;
                        loaded.reserve(obs_nodes.size());
                        for (std::size_t i = 0; i < obs_nodes.size(); ++i) {
                            const YAML::Node& n = obs_nodes[i];
                            UserObstacleItem item;
                            if (n["name"])
                                item.name = n["name"].as<std::string>();
                            if (item.name.empty())
                                item.name = "obs_" + std::to_string(i + 1);
                            if (n["kind"])
                                item.kind = ObstacleKindFromName(n["kind"].as<std::string>());
                            if (n["visible"])
                                item.visible = n["visible"].as<bool>();
                            if (n["color"] && n["color"].IsSequence() && n["color"].size() >= 3) {
                                item.color = glm::vec3(n["color"][0].as<float>(), n["color"][1].as<float>(), n["color"][2].as<float>());
                            }
                            if (n["position"] && n["position"].IsSequence() && n["position"].size() >= 3) {
                                item.position =
                                    glm::vec3(n["position"][0].as<float>(), n["position"][1].as<float>(), n["position"][2].as<float>());
                            }
                            if (n["rpy_deg"] && n["rpy_deg"].IsSequence() && n["rpy_deg"].size() >= 3) {
                                item.rpy_deg =
                                    glm::vec3(n["rpy_deg"][0].as<float>(), n["rpy_deg"][1].as<float>(), n["rpy_deg"][2].as<float>());
                            }
                            if (n["params"] && n["params"].IsSequence() && n["params"].size() >= 3) {
                                item.params = glm::vec3(n["params"][0].as<float>(), n["params"][1].as<float>(), n["params"][2].as<float>());
                            }
                            loaded.push_back(item);
                        }
                        st->items          = std::move(loaded);
                        st->selected_index = st->items.empty() ? -1 : 0;
                        try_update_next_serial();
                        import_status = std::string("导入成功: ") + import_path + " (共 " + std::to_string(st->items.size()) + " 个)";
                    }
                } catch (const std::exception& e) {
                    import_status = std::string("导入失败: ") + e.what();
                }
            }
            if (!import_status.empty()) {
                const bool ok = import_status.find("成功") != std::string::npos;
                ImGui::TextColored(ok ? ImVec4(0.60f, 0.92f, 0.60f, 1.0f) : ImVec4(1.0f, 0.45f, 0.45f, 1.0f), "%s", import_status.c_str());
            }
        }
        ImGui::Separator();
        if (ImGui::CollapsingHeader("新建障碍物", ImGuiTreeNodeFlags_DefaultOpen)) {
            static bool draft_initialized = false;
            static int draft_kind_index   = static_cast<int>(UserObstacleItem::Kind::Box);
            static UserObstacleItem draft;
            static char draft_name[128]       = "";
            static char draft_pose_input[256] = "";
            static std::string draft_pose_status;

            auto reset_draft_for_kind = [&](int kind_index) {
                draft      = UserObstacleItem{};
                draft.kind = static_cast<UserObstacleItem::Kind>(kind_index);
                if (draft.kind == UserObstacleItem::Kind::Sphere) {
                    draft.params = glm::vec3(0.15f, 0.0f, 0.0f);
                } else if (draft.kind == UserObstacleItem::Kind::Cylinder) {
                    draft.params = glm::vec3(0.12f, 0.45f, 0.0f);
                }
                draft.position              = glm::vec3(0.45f, 0.0f, 0.35f);
                draft.rpy_deg               = glm::vec3(0.0f);
                draft.color                 = glm::vec3(0.82f, 0.52f, 0.18f);
                draft.visible               = true;
                draft_name[0]               = '\0';
                const glm::quat q           = glm::normalize(glm::quat_cast(markerWorldMatrix(glm::vec3(0.0f), draft.rpy_deg)));
                const std::string pose_text = FormatPoseInputXyzQuat(draft.position, q);
                std::snprintf(draft_pose_input, sizeof(draft_pose_input), "%s", pose_text.c_str());
                draft_pose_status.clear();
            };

            if (!draft_initialized) {
                reset_draft_for_kind(draft_kind_index);
                draft_initialized = true;
            }

            const char* kind_labels[] = {"盒子", "球体", "圆柱"};
            int prev_kind             = draft_kind_index;
            ImGui::Combo("类型", &draft_kind_index, kind_labels, IM_ARRAYSIZE(kind_labels));
            if (prev_kind != draft_kind_index) {
                reset_draft_for_kind(draft_kind_index);
            }
            ImGui::InputText("名称(可选)", draft_name, sizeof(draft_name));
            ImGui::InputText("位姿串(新建 x,y,z,qx,qy,qz,qw)", draft_pose_input, sizeof(draft_pose_input));
            if (ImGui::Button("填充当前新建位姿串")) {
                const glm::quat q           = glm::normalize(glm::quat_cast(markerWorldMatrix(glm::vec3(0.0f), draft.rpy_deg)));
                const std::string pose_text = FormatPoseInputXyzQuat(draft.position, q);
                std::snprintf(draft_pose_input, sizeof(draft_pose_input), "%s", pose_text.c_str());
                draft_pose_status.clear();
            }
            ImGui::SameLine();
            if (ImGui::Button("应用位姿串到新建")) {
                glm::vec3 parsed_pos(0.0f);
                glm::quat parsed_quat(1.0f, 0.0f, 0.0f, 0.0f);
                std::string parse_error;
                if (ParsePoseInputXyzQuat(draft_pose_input, &parsed_pos, &parsed_quat, &parse_error)) {
                    draft.position    = parsed_pos;
                    draft.rpy_deg     = glm::degrees(glm::eulerAngles(parsed_quat));
                    draft_pose_status = "新建位姿串应用成功";
                } else {
                    draft_pose_status = std::string("新建位姿串应用失败: ") + parse_error;
                }
            }
            if (!draft_pose_status.empty()) {
                const bool ok = draft_pose_status.find("成功") != std::string::npos;
                ImGui::TextColored(ok ? ImVec4(0.60f, 0.92f, 0.60f, 1.0f) : ImVec4(1.0f, 0.45f, 0.45f, 1.0f), "%s",
                                   draft_pose_status.c_str());
            }
            ImGui::ColorEdit3("颜色(新建)", glm::value_ptr(draft.color));
            ImGui::DragFloat3("位置(m, 新建)", glm::value_ptr(draft.position), 0.005f, -10.0f, 10.0f, "%.3f");
            {
                char draft_rpy_label[40];
                std::snprintf(draft_rpy_label, sizeof(draft_rpy_label), "姿态rpy(%s, 新建)", AngleUnitLabel(angle_unit_deg));
                draw_rpy_drag(draft_rpy_label, &draft.rpy_deg);
            }
            if (draft.kind == UserObstacleItem::Kind::Box) {
                ImGui::DragFloat3("长宽高(m, 新建)", glm::value_ptr(draft.params), 0.005f, 0.02f, 5.0f, "%.3f");
            } else if (draft.kind == UserObstacleItem::Kind::Sphere) {
                ImGui::DragFloat("半径(m, 新建)", &draft.params.x, 0.005f, 0.02f, 3.0f, "%.3f");
            } else {
                ImGui::DragFloat("半径(m, 新建)", &draft.params.x, 0.005f, 0.02f, 3.0f, "%.3f");
                ImGui::DragFloat("高度(m, 本地Y, 新建)", &draft.params.y, 0.005f, 0.02f, 5.0f, "%.3f");
            }
            if (ImGui::Button("确认添加")) {
                capture_undo("新增障碍物");
                UserObstacleItem o = draft;
                char auto_name[64];
                if (draft_name[0] == '\0') {
                    const char* prefix = "box";
                    if (o.kind == UserObstacleItem::Kind::Sphere) {
                        prefix = "sphere";
                    } else if (o.kind == UserObstacleItem::Kind::Cylinder) {
                        prefix = "cyl";
                    }
                    std::snprintf(auto_name, sizeof(auto_name), "%s_%d", prefix, st->next_serial++);
                    o.name = auto_name;
                } else {
                    o.name = draft_name;
                    st->next_serial++;
                }
                st->items.push_back(std::move(o));
                st->selected_index = static_cast<int>(st->items.size()) - 1;
                reset_draft_for_kind(draft_kind_index);
            }
            ImGui::SameLine();
            if (ImGui::Button("重置新建参数")) {
                reset_draft_for_kind(draft_kind_index);
            }

            ImGui::Separator();
        }
        if (ImGui::CollapsingHeader("已添加障碍物", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::Button("复制选中") && st->selected_index >= 0 && st->selected_index < static_cast<int>(st->items.size())) {
                capture_undo("复制选中");
                UserObstacleItem copied = st->items[static_cast<size_t>(st->selected_index)];
                copied.position += glm::vec3(0.03f, 0.03f, 0.0f);
                char auto_name[64];
                std::snprintf(auto_name, sizeof(auto_name), "%s_copy_%d", copied.name.c_str(), st->next_serial++);
                copied.name = auto_name;
                st->items.push_back(std::move(copied));
                st->selected_index = static_cast<int>(st->items.size()) - 1;
            }
            ImGui::SameLine();
            if (ImGui::Button("删除选中") && st->selected_index >= 0 && st->selected_index < static_cast<int>(st->items.size())) {
                capture_undo("删除选中");
                st->items.erase(st->items.begin() + st->selected_index);
                st->selected_index = std::min(st->selected_index, static_cast<int>(st->items.size()) - 1);
            }
            ImGui::SameLine();
            if (ImGui::Button("清空全部") && !st->items.empty()) {
                ImGui::OpenPopup("confirm_clear_all_obstacles");
            }
            if (ImGui::BeginPopupModal("confirm_clear_all_obstacles", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
                ImGui::Text("确定要清空全部障碍物吗？");
                ImGui::TextDisabled("该操作不可撤销");
                if (ImGui::Button("确定清空")) {
                    capture_undo("清空全部");
                    st->items.clear();
                    st->selected_index = -1;
                    ImGui::CloseCurrentPopup();
                }
                ImGui::SameLine();
                if (ImGui::Button("取消")) {
                    ImGui::CloseCurrentPopup();
                }
                ImGui::EndPopup();
            }

            if (!st->items.empty()) {
                ImGui::Separator();
                static char obstacle_filter[128] = "";
                ImGui::InputText("列表过滤", obstacle_filter, sizeof(obstacle_filter));
                std::string filter = obstacle_filter;
                std::transform(filter.begin(), filter.end(), filter.begin(),
                               [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
                std::vector<const char*> label_ptrs;
                std::vector<int> visible_indices;
                label_ptrs.reserve(st->items.size());
                visible_indices.reserve(st->items.size());
                int selected_visible_index = -1;
                for (int idx = 0; idx < static_cast<int>(st->items.size()); ++idx) {
                    const auto& it  = st->items[static_cast<size_t>(idx)];
                    std::string key = it.name;
                    std::transform(key.begin(), key.end(), key.begin(),
                                   [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
                    if (!filter.empty() && key.find(filter) == std::string::npos) {
                        continue;
                    }
                    label_ptrs.push_back(it.name.c_str());
                    visible_indices.push_back(idx);
                    if (idx == st->selected_index) {
                        selected_visible_index = static_cast<int>(label_ptrs.size()) - 1;
                    }
                }
                if (label_ptrs.empty()) {
                    ImGui::TextDisabled("没有匹配的障碍物");
                } else {
                    if (selected_visible_index < 0) {
                        selected_visible_index = 0;
                    }
                    if (ImGui::ListBox("障碍物列表", &selected_visible_index, label_ptrs.data(), static_cast<int>(label_ptrs.size()), 6)) {
                        if (selected_visible_index >= 0 && selected_visible_index < static_cast<int>(visible_indices.size())) {
                            st->selected_index = visible_indices[static_cast<size_t>(selected_visible_index)];
                        }
                    }
                }
            }

            if (st->selected_index >= 0 && st->selected_index < static_cast<int>(st->items.size())) {
                UserObstacleItem& o                  = st->items[static_cast<size_t>(st->selected_index)];
                static int last_selected_index       = -1;
                static char selected_pose_input[256] = "";
                static std::string selected_pose_status;
                if (last_selected_index != st->selected_index) {
                    const glm::quat q           = glm::normalize(glm::quat_cast(markerWorldMatrix(glm::vec3(0.0f), o.rpy_deg)));
                    const std::string pose_text = FormatPoseInputXyzQuat(o.position, q);
                    std::snprintf(selected_pose_input, sizeof(selected_pose_input), "%s", pose_text.c_str());
                    selected_pose_status.clear();
                    last_selected_index = st->selected_index;
                }
                ImGui::Separator();
                ImGui::Checkbox("显示", &o.visible);
                char name_buf[128];
                std::snprintf(name_buf, sizeof(name_buf), "%s", o.name.c_str());
                if (ImGui::InputText("名称", name_buf, sizeof(name_buf))) {
                    o.name = name_buf;
                }
                ImGui::InputText("位姿串(选中 x,y,z,qx,qy,qz,qw)", selected_pose_input, sizeof(selected_pose_input));
                if (ImGui::Button("填充当前选中位姿串")) {
                    const glm::quat q           = glm::normalize(glm::quat_cast(markerWorldMatrix(glm::vec3(0.0f), o.rpy_deg)));
                    const std::string pose_text = FormatPoseInputXyzQuat(o.position, q);
                    std::snprintf(selected_pose_input, sizeof(selected_pose_input), "%s", pose_text.c_str());
                    selected_pose_status.clear();
                }
                ImGui::SameLine();
                if (ImGui::Button("应用位姿串到选中")) {
                    glm::vec3 parsed_pos(0.0f);
                    glm::quat parsed_quat(1.0f, 0.0f, 0.0f, 0.0f);
                    std::string parse_error;
                    if (ParsePoseInputXyzQuat(selected_pose_input, &parsed_pos, &parsed_quat, &parse_error)) {
                        capture_undo("应用选中位姿串");
                        o.position           = parsed_pos;
                        o.rpy_deg            = glm::degrees(glm::eulerAngles(parsed_quat));
                        selected_pose_status = "选中位姿串应用成功";
                    } else {
                        selected_pose_status = std::string("选中位姿串应用失败: ") + parse_error;
                    }
                }
                if (!selected_pose_status.empty()) {
                    const bool ok = selected_pose_status.find("成功") != std::string::npos;
                    ImGui::TextColored(ok ? ImVec4(0.60f, 0.92f, 0.60f, 1.0f) : ImVec4(1.0f, 0.45f, 0.45f, 1.0f), "%s",
                                       selected_pose_status.c_str());
                }
                ImGui::ColorEdit3("颜色", glm::value_ptr(o.color));
                ImGui::DragFloat3("位置 (m)", glm::value_ptr(o.position), 0.005f, -10.0f, 10.0f, "%.3f");
                {
                    char selected_rpy_label[40];
                    std::snprintf(selected_rpy_label, sizeof(selected_rpy_label), "姿态 rpy (%s)", AngleUnitLabel(angle_unit_deg));
                    draw_rpy_drag(selected_rpy_label, &o.rpy_deg);
                }
                if (o.kind == UserObstacleItem::Kind::Box) {
                    ImGui::DragFloat3("长宽高 (m)", glm::value_ptr(o.params), 0.005f, 0.02f, 5.0f, "%.3f");
                } else if (o.kind == UserObstacleItem::Kind::Sphere) {
                    ImGui::DragFloat("半径 (m)", &o.params.x, 0.005f, 0.02f, 3.0f, "%.3f");
                } else {
                    ImGui::DragFloat("半径 (m)", &o.params.x, 0.005f, 0.02f, 3.0f, "%.3f");
                    ImGui::DragFloat("高度 (m, 本地Y)", &o.params.y, 0.005f, 0.02f, 5.0f, "%.3f");
                }
            }
        }
    }

    void MergeUserObstaclesIntoCollisionResult(const UserObstacleState& obstacles, const teleop_viewer::RobotScene& scene,
                                               float warning_distance_m, float danger_distance_m, CollisionMonitorResult* result) {
        if (result == nullptr || !obstacles.affect_collision) {
            return;
        }

        const auto proxies = scene.getLinkCollisionProxies();
        if (proxies.empty()) {
            return;
        }

        float best_surface         = result->valid ? result->closest_pair.surface_distance_m : 1e9f;
        CollisionPairDistance best = result->closest_pair;
        bool has_best              = result->valid;
        int add_eval               = 0;
        int add_warn               = 0;
        int add_dang               = 0;

        for (const auto& obs : obstacles.items) {
            if (!obs.visible) {
                continue;
            }
            for (const auto& link : proxies) {
                float surf = 0.0f;
                float ctr  = 0.0f;
                glm::vec3 pa;
                glm::vec3 pb;
                evalLinkVsObstacle(link, obs, &surf, &ctr, &pa, &pb);
                ++add_eval;
                if (surf <= warning_distance_m + 1e-6f) {
                    ++add_warn;
                }
                if (surf <= danger_distance_m + 1e-6f) {
                    ++add_dang;
                }
                if (!has_best || surf < best_surface - 1e-9f) {
                    has_best                = true;
                    best_surface            = surf;
                    best.link_a             = link.link_name;
                    best.link_b             = std::string("[障碍] ") + obs.name;
                    best.point_a            = pa;
                    best.point_b            = pb;
                    best.center_distance_m  = ctr;
                    best.surface_distance_m = surf;
                }
            }
        }

        result->evaluated_pair_count += add_eval;
        result->warning_pair_count += add_warn;
        result->danger_pair_count += add_dang;

        if (has_best) {
            result->valid        = true;
            result->closest_pair = best;
        }
    }

}  // namespace kinematic_viewer
