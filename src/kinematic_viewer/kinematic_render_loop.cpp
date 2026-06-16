#include "kinematic_viewer/kinematic_render_loop.h"

#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

namespace kinematic_viewer {

    void KinematicRenderLoop::Render(const Context& ctx) {
        if (!ctx.scene || !ctx.ui_state || !ctx.ik_state || !ctx.collision_state || !ctx.collision_result || !ctx.obstacle_meshes ||
            !ctx.camera || !line_renderer) {
            return;
        }

        SetupViewportAndClear(ctx);

        const glm::mat4 proj =
            glm::perspective(glm::radians(50.0f), static_cast<float>(ctx.viewport_w) / static_cast<float>(ctx.viewport_h), 0.05f, 80.0f);
        const glm::mat4 view = ctx.camera->viewMatrix();
        const glm::vec3 eye  = ctx.camera->eye();

        DrawPointCloud(ctx, view, proj);
        DrawSceneMeshes(ctx.mesh_shader, ctx, proj, view);
        DrawObstacles(ctx.mesh_shader, ctx, view, proj);

        std::vector<KinematicLineVertex> axis_vertices;
        BuildAxisVertices(ctx, &axis_vertices);
        DrawLineOverlays(ctx.line_shader, ctx, axis_vertices, view, proj);
    }

    void KinematicRenderLoop::SetupViewportAndClear(const Context& ctx) {
        (void)ctx;
        glViewport(0, 0, ctx.viewport_w, ctx.viewport_h);
        glEnable(GL_DEPTH_TEST);
        // Newton ViewerGL sky_lower — dark viewport backdrop.
        glClearColor(40.0f / 255.0f, 44.0f / 255.0f, 55.0f / 255.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }

    void KinematicRenderLoop::SetupMeshShaderUniforms(GLuint shader, const glm::mat4& proj, const glm::mat4& view, const glm::vec3& eye) {
        glUseProgram(shader);
        glUniformMatrix4fv(glGetUniformLocation(shader, "projection"), 1, GL_FALSE, glm::value_ptr(proj));
        glUniformMatrix4fv(glGetUniformLocation(shader, "view"), 1, GL_FALSE, glm::value_ptr(view));
        glUniform3f(glGetUniformLocation(shader, "viewPos"), eye.x, eye.y, eye.z);
        // Z-up sun direction (Newton ViewerGL default for up_axis=2).
        const glm::vec3 sun_dir = glm::normalize(glm::vec3(0.2f, -0.3f, 0.8f));
        glUniform3f(glGetUniformLocation(shader, "sunDirection"), sun_dir.x, sun_dir.y, sun_dir.z);
        glUniform3f(glGetUniformLocation(shader, "lightColor"), 1.8f, 1.8f, 1.8f);
        glUniform3f(glGetUniformLocation(shader, "skyColor"), 0.745f, 0.863f, 0.941f);
        glUniform3f(glGetUniformLocation(shader, "groundColor"), 0.294f, 0.333f, 0.592f);
        glUniform1f(glGetUniformLocation(shader, "diffuseScale"), 1.0f);
        glUniform1f(glGetUniformLocation(shader, "specularScale"), 1.0f);
        glUniform1i(glGetUniformLocation(shader, "upAxis"), 2);
    }

    void KinematicRenderLoop::DrawSceneMeshes(GLuint shader, const Context& ctx, const glm::mat4& proj, const glm::mat4& view) {
        SetupMeshShaderUniforms(shader, proj, view, ctx.camera->eye());

        rkv::RobotScene::SceneDrawStyle style;
        if (ctx.ui_state != nullptr) {
            style.show_visual_meshes    = ctx.ui_state->show_visual_meshes;
            style.show_collision_bodies = ctx.ui_state->show_collision_bodies;
            style.wireframe_visuals     = ctx.ui_state->show_wireframe;
            style.hovered_link = ctx.ui_state->hovered_link;
            if (ctx.ui_state->enable_link_click_select) {
                style.selected_link = ctx.ui_state->selected_link;
            }
        }

        if (style.show_collision_bodies) {
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glDepthMask(GL_TRUE);
        }

        ctx.scene->draw(shader, style);

        if (style.show_collision_bodies) {
            glDisable(GL_BLEND);
        }
    }

    void KinematicRenderLoop::DrawObstacles(GLuint shader, const Context& ctx, const glm::mat4& view, const glm::mat4& proj) {
        DrawUserObstacles(shader, ctx.ui_state->user_obstacles, *ctx.obstacle_meshes, view, proj);
    }

    void KinematicRenderLoop::DrawPointCloud(const Context& ctx, const glm::mat4& view, const glm::mat4& proj) {
        if (ctx.point_shader == 0 || ctx.point_cloud == nullptr || ctx.point_cloud_layer == nullptr) {
            return;
        }
        if (!ctx.point_cloud->visible || !ctx.point_cloud_layer->hasGpuData()) {
            return;
        }

        glm::mat4 model(1.0f);
        model = glm::translate(model, glm::vec3(ctx.point_cloud->offset_x, ctx.point_cloud->offset_y, 0.0f));
        model = glm::rotate(model, glm::radians(ctx.point_cloud->offset_yaw), glm::vec3(0.0f, 0.0f, 1.0f));

        ctx.point_cloud_layer->Draw(ctx.point_shader, view, proj, model, ctx.point_cloud->point_size_px);
    }

    void KinematicRenderLoop::BuildAxisVertices(const Context& ctx, std::vector<KinematicLineVertex>* out) {
        BuildGridLines(ctx, out);
        BuildWorldAxes(ctx, out);
        BuildMobileBaseAxes(ctx, out);
        BuildJointAxes(ctx, out);
        BuildMarkerAxes(ctx, out);
        BuildCollisionLine(ctx, out);
        BuildPlannedPathLines(ctx, out);
        BuildSelectedLinkHighlight(ctx, out);
        if (ctx.ui_state->show_com && ctx.scene != nullptr) {
            const glm::vec3 com_color(0.95f, 0.35f, 0.15f);
            const float com_size = 0.018f;
            for (const auto& com : ctx.scene->getLinkComWorldPositions()) {
                const glm::vec3& p = com.world_position;
                out->push_back({p + glm::vec3(-com_size, 0.0f, 0.0f), com_color});
                out->push_back({p + glm::vec3(com_size, 0.0f, 0.0f), com_color});
                out->push_back({p + glm::vec3(0.0f, -com_size, 0.0f), com_color});
                out->push_back({p + glm::vec3(0.0f, com_size, 0.0f), com_color});
                out->push_back({p + glm::vec3(0.0f, 0.0f, -com_size), com_color});
                out->push_back({p + glm::vec3(0.0f, 0.0f, com_size), com_color});
            }
        }

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
        if (ctx.ui_state == nullptr || !ctx.ui_state->show_grid) {
            return;
        }
        float half = ctx.ui_state->grid_size;
        int count  = std::max(2, ctx.ui_state->grid_count);
        glm::vec3 grid_col(0.30f, 0.34f, 0.42f);
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

    void KinematicRenderLoop::BuildMobileBaseAxes(const Context& ctx, std::vector<KinematicLineVertex>* out) {
        if (ctx.ui_state == nullptr || ctx.scene == nullptr || !ctx.ui_state->mobile_base_drag_enabled) {
            return;
        }
        float base_x_m = 0.0f;
        float base_y_m = 0.0f;
        float base_yaw = 0.0f;
        if (!ctx.scene->getVirtualBasePose2D(&base_x_m, &base_y_m, &base_yaw)) {
            return;
        }
        const glm::vec3 base_pos(base_x_m, base_y_m, 0.0f);
        const glm::vec3 base_rpy_deg(0.0f, 0.0f, glm::degrees(base_yaw));
        appendMarkerAxes(out, base_pos, base_rpy_deg, 0.22f, true);
        const glm::vec3 ring_color(0.15f, 0.95f, 0.55f);
        appendCircle(out, base_pos, glm::vec3(0.0f, 0.0f, 1.0f), 0.25f, ring_color, 48);
    }

    void KinematicRenderLoop::BuildMarkerAxes(const Context& ctx, std::vector<KinematicLineVertex>* out) {
        if (ctx.ui_state != nullptr && ctx.ui_state->mobile_base_drag_enabled && ctx.ui_state->scene_panel_active) {
            return;
        }
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

    void KinematicRenderLoop::BuildSelectedLinkHighlight(const Context& ctx, std::vector<KinematicLineVertex>* out) {
        if (ctx.ui_state == nullptr || ctx.scene == nullptr) {
            return;
        }

        auto drawLinkRing = [&](const std::string& link_name, const glm::vec3& color) {
            if (link_name.empty()) {
                return;
            }
            for (const auto& proxy : ctx.scene->getLinkCollisionProxies()) {
                if (proxy.link_name != link_name) {
                    continue;
                }
                const float pick_r = std::max(proxy.radius_m * 1.08f, 0.025f);
                appendCircle(out, proxy.world_center, glm::vec3(1.0f, 0.0f, 0.0f), pick_r, color, 36);
                appendCircle(out, proxy.world_center, glm::vec3(0.0f, 1.0f, 0.0f), pick_r, color, 36);
                appendCircle(out, proxy.world_center, glm::vec3(0.0f, 0.0f, 1.0f), pick_r, color, 36);
                break;
            }
        };

        if (ctx.ui_state->enable_link_hover_highlight && !ctx.ui_state->hovered_link.empty() &&
            ctx.ui_state->hovered_link != ctx.ui_state->selected_link) {
            drawLinkRing(ctx.ui_state->hovered_link, glm::vec3(1.0f, 0.78f, 0.22f));
        }
        if (ctx.ui_state->enable_link_click_select && !ctx.ui_state->selected_link.empty()) {
            drawLinkRing(ctx.ui_state->selected_link, glm::vec3(0.25f, 0.92f, 1.0f));
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
