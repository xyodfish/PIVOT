#include "kinematic_viewer/kinematic_render_loop.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

namespace kinematic_viewer {

    void KinematicRenderLoop::Render(const Context& ctx) {
        if (!ctx.scene || !ctx.ui_state || !ctx.ik_state || !ctx.collision_state || !ctx.collision_result || !ctx.obstacle_meshes ||
            !ctx.camera || !line_renderer) {
            return;
        }

        SetupViewportAndClear(ctx.viewport_w, ctx.viewport_h);

        const glm::mat4 proj =
            glm::perspective(glm::radians(50.0f), static_cast<float>(ctx.viewport_w) / static_cast<float>(ctx.viewport_h), 0.05f, 80.0f);
        const glm::mat4 view = ctx.camera->viewMatrix();
        const glm::vec3 eye  = ctx.camera->eye();

        DrawSceneMeshes(ctx.mesh_shader, ctx, proj, view);
        DrawObstacles(ctx.mesh_shader, ctx, view, proj);

        std::vector<KinematicLineVertex> axis_vertices;
        BuildAxisVertices(ctx, &axis_vertices);
        DrawLineOverlays(ctx.line_shader, ctx, axis_vertices, view, proj);
    }

    void KinematicRenderLoop::SetupViewportAndClear(int w, int h) {
        glViewport(0, 0, w, h);
        glEnable(GL_DEPTH_TEST);
        glClearColor(0.90f, 0.92f, 0.96f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }

    void KinematicRenderLoop::SetupMeshShaderUniforms(GLuint shader, const glm::mat4& proj, const glm::mat4& view, const glm::vec3& eye) {
        glUseProgram(shader);
        glUniformMatrix4fv(glGetUniformLocation(shader, "projection"), 1, GL_FALSE, glm::value_ptr(proj));
        glUniformMatrix4fv(glGetUniformLocation(shader, "view"), 1, GL_FALSE, glm::value_ptr(view));
        const glm::vec3 light_pos = eye + glm::vec3(0.8f, 0.8f, 1.2f);
        glUniform3f(glGetUniformLocation(shader, "lightPos"), light_pos.x, light_pos.y, light_pos.z);
        glUniform3f(glGetUniformLocation(shader, "viewPos"), eye.x, eye.y, eye.z);
    }

    void KinematicRenderLoop::DrawSceneMeshes(GLuint shader, const Context& ctx, const glm::mat4& proj, const glm::mat4& view) {
        SetupMeshShaderUniforms(shader, proj, view, ctx.camera->eye());
        ctx.scene->draw(shader);
    }

    void KinematicRenderLoop::DrawObstacles(GLuint shader, const Context& ctx, const glm::mat4& view, const glm::mat4& proj) {
        DrawUserObstacles(shader, ctx.ui_state->user_obstacles, *ctx.obstacle_meshes, view, proj);
    }

    void KinematicRenderLoop::BuildAxisVertices(const Context& ctx, std::vector<KinematicLineVertex>* out) {
        BuildGridLines(ctx, out);
        BuildWorldAxes(ctx, out);
        BuildJointAxes(ctx, out);
        BuildMarkerAxes(ctx, out);
        BuildCollisionLine(ctx, out);
        BuildPlannedPathLines(ctx, out);

        // Selected obstacle highlight axes and pick circles
        if (ctx.ui_state->user_obstacles.selected_index >= 0 &&
            ctx.ui_state->user_obstacles.selected_index < static_cast<int>(ctx.ui_state->user_obstacles.items.size())) {
            const auto& selected_obs = ctx.ui_state->user_obstacles.items[static_cast<size_t>(ctx.ui_state->user_obstacles.selected_index)];
            if (selected_obs.visible) {
                const glm::vec3 hi_color(0.10f, 0.85f, 1.0f);
                appendMarkerAxes(out, selected_obs.position, selected_obs.rpy_deg, 0.18f, true);
                const float pick_r = [&]() -> float {
                    if (selected_obs.kind == UserObstacleItem::Kind::Sphere) {
                        return std::max(1e-4f, selected_obs.params.x);
                    }
                    if (selected_obs.kind == UserObstacleItem::Kind::Box) {
                        return 0.5f * glm::length(glm::vec3(std::max(1e-4f, selected_obs.params.x), std::max(1e-4f, selected_obs.params.y),
                                                            std::max(1e-4f, selected_obs.params.z)));
                    }
                    const float r = std::max(1e-4f, selected_obs.params.x);
                    const float h = std::max(1e-4f, selected_obs.params.y);
                    return std::sqrt(r * r + 0.25f * h * h);
                }();
                appendCircle(out, selected_obs.position, glm::vec3(1.0f, 0.0f, 0.0f), pick_r, hi_color, 36);
                appendCircle(out, selected_obs.position, glm::vec3(0.0f, 1.0f, 0.0f), pick_r, hi_color, 36);
                appendCircle(out, selected_obs.position, glm::vec3(0.0f, 0.0f, 1.0f), pick_r, hi_color, 36);
            }
        }
    }

    void KinematicRenderLoop::BuildGridLines(const Context& ctx, std::vector<KinematicLineVertex>* out) {
        float half = ctx.ui_state->grid_size;
        int count  = std::max(2, ctx.ui_state->grid_count);
        glm::vec3 grid_col(0.72f, 0.76f, 0.82f);
        for (int i = 0; i <= count; ++i) {
            float t = -half + 2.0f * half * (static_cast<float>(i) / static_cast<float>(count));
            out->push_back({glm::vec3(-half, t, 0.0f), grid_col});
            out->push_back({glm::vec3(half, t, 0.0f), grid_col});
            out->push_back({glm::vec3(t, -half, 0.0f), grid_col});
            out->push_back({glm::vec3(t, half, 0.0f), grid_col});
        }
    }

    void KinematicRenderLoop::BuildWorldAxes(const Context& ctx, std::vector<KinematicLineVertex>* out) {
        if (!ctx.ui_state->show_world_axes) {
            return;
        }
        glm::vec3 o(0.0f);
        float l = ctx.ui_state->world_axis_length;
        out->push_back({o, glm::vec3(1, 0, 0)});
        out->push_back({o + glm::vec3(l, 0, 0), glm::vec3(1, 0, 0)});
        out->push_back({o, glm::vec3(0, 1, 0)});
        out->push_back({o + glm::vec3(0, l, 0), glm::vec3(0, 1, 0)});
        out->push_back({o, glm::vec3(0, 0, 1)});
        out->push_back({o + glm::vec3(0, 0, l), glm::vec3(0, 0, 1)});
    }

    void KinematicRenderLoop::BuildJointAxes(const Context& ctx, std::vector<KinematicLineVertex>* out) {
        if (!ctx.ui_state->show_axes) {
            return;
        }
        auto axes = ctx.scene->getJointAxisInfos(ctx.ui_state->show_revolute_only);
        for (const auto& a : axes) {
            if (!ctx.ui_state->show_non_revolute && !a.revolute) {
                continue;
            }
            glm::vec3 c  = a.revolute ? glm::vec3(1.0f, 0.82f, 0.1f) : glm::vec3(0.45f, 0.8f, 1.0f);
            glm::vec3 p0 = a.world_origin - 0.5f * ctx.ui_state->axis_length * a.world_axis;
            glm::vec3 p1 = a.world_origin + 0.5f * ctx.ui_state->axis_length * a.world_axis;
            out->push_back({p0, c});
            out->push_back({p1, c});
        }
    }

    void KinematicRenderLoop::BuildMarkerAxes(const Context& ctx, std::vector<KinematicLineVertex>* out) {
        for (int i = 0; i < static_cast<int>(ctx.ik_state->marker_targets.size()); ++i) {
            const auto& target = ctx.ik_state->marker_targets[i];
            if (!target.initialized) {
                continue;
            }
            appendMarkerAxes(out, target.pos, target.rpy_deg, 0.12f, i == ctx.ik_state->selected_chain);
        }
    }

    void KinematicRenderLoop::BuildPlannedPathLines(const Context& ctx, std::vector<KinematicLineVertex>* out) {
        if (!ctx.show_planned_path || ctx.planned_path == nullptr || ctx.planned_path->size() < 2) {
            return;
        }
        const glm::vec3 path_color(1.0f, 0.35f, 0.85f);       // Magenta-ish for path
        const glm::vec3 waypoint_color(0.95f, 0.95f, 0.25f);  // Yellow for waypoints
        const float waypoint_size = 0.008f;

        // Draw path line segments
        for (size_t i = 1; i < ctx.planned_path->size(); ++i) {
            const glm::vec3& p0 = (*ctx.planned_path)[i - 1].position;
            const glm::vec3& p1 = (*ctx.planned_path)[i].position;
            out->push_back({p0, path_color});
            out->push_back({p1, path_color});
        }

        // Draw small crosses at each waypoint
        for (const auto& wp : *ctx.planned_path) {
            const glm::vec3& p = wp.position;
            out->push_back({p + glm::vec3(-waypoint_size, 0.0f, 0.0f), waypoint_color});
            out->push_back({p + glm::vec3(waypoint_size, 0.0f, 0.0f), waypoint_color});
            out->push_back({p + glm::vec3(0.0f, -waypoint_size, 0.0f), waypoint_color});
            out->push_back({p + glm::vec3(0.0f, waypoint_size, 0.0f), waypoint_color});
            out->push_back({p + glm::vec3(0.0f, 0.0f, -waypoint_size), waypoint_color});
            out->push_back({p + glm::vec3(0.0f, 0.0f, waypoint_size), waypoint_color});
        }
    }

    void KinematicRenderLoop::BuildCollisionLine(const Context& ctx, std::vector<KinematicLineVertex>* out) {
        if (!ctx.collision_state->enable || !ctx.collision_state->show_closest_pair_line || !ctx.collision_state->has_valid_distance) {
            return;
        }
        glm::vec3 line_color(0.20f, 0.95f, 0.20f);
        if (ctx.collision_state->nearest_surface_distance_m <= ctx.collision_state->danger_distance_m) {
            line_color = glm::vec3(1.0f, 0.25f, 0.25f);
        } else if (ctx.collision_state->nearest_surface_distance_m <= ctx.collision_state->warning_distance_m) {
            line_color = glm::vec3(1.0f, 0.75f, 0.25f);
        }
        out->push_back({ctx.collision_state->nearest_point_a, line_color});
        out->push_back({ctx.collision_state->nearest_point_b, line_color});
    }

    void KinematicRenderLoop::DrawLineOverlays(GLuint shader, const Context& ctx, const std::vector<KinematicLineVertex>& vertices,
                                               const glm::mat4& view, const glm::mat4& proj) {
        line_renderer->draw(shader, vertices, view, proj, ctx.ui_state->axis_line_width);
    }

}  // namespace kinematic_viewer
