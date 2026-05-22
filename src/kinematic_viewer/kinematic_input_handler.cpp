#include "kinematic_viewer/kinematic_input_handler.h"

#include "imgui.h"

#include <glm/gtc/matrix_transform.hpp>

#include <cmath>

namespace kinematic_viewer {

    namespace {

        bool ComputeWorldRayFromScreen(float mouse_x, float mouse_y, int viewport_w, int viewport_h, const glm::mat4& view,
                                       const glm::mat4& proj, glm::vec3* out_origin, glm::vec3* out_dir) {
            if (out_origin == nullptr || out_dir == nullptr || viewport_w <= 0 || viewport_h <= 0) {
                return false;
            }
            const float x_ndc = (2.0f * mouse_x) / static_cast<float>(viewport_w) - 1.0f;
            const float y_ndc = 1.0f - (2.0f * mouse_y) / static_cast<float>(viewport_h);
            const glm::vec4 near_clip(x_ndc, y_ndc, -1.0f, 1.0f);
            const glm::vec4 far_clip(x_ndc, y_ndc, 1.0f, 1.0f);
            const glm::mat4 inv_vp = glm::inverse(proj * view);
            glm::vec4 near_world4  = inv_vp * near_clip;
            glm::vec4 far_world4   = inv_vp * far_clip;
            if (std::fabs(near_world4.w) < 1e-8f || std::fabs(far_world4.w) < 1e-8f) {
                return false;
            }
            glm::vec3 near_world = glm::vec3(near_world4) / near_world4.w;
            glm::vec3 far_world  = glm::vec3(far_world4) / far_world4.w;
            glm::vec3 dir        = far_world - near_world;
            if (glm::length(dir) < 1e-8f) {
                return false;
            }
            *out_origin = near_world;
            *out_dir    = glm::normalize(dir);
            return true;
        }

        float ObstaclePickRadius(const UserObstacleItem& obs) {
            if (obs.kind == UserObstacleItem::Kind::Sphere) {
                return std::max(1e-4f, obs.params.x);
            }
            if (obs.kind == UserObstacleItem::Kind::Box) {
                return 0.5f *
                       glm::length(glm::vec3(std::max(1e-4f, obs.params.x), std::max(1e-4f, obs.params.y), std::max(1e-4f, obs.params.z)));
            }
            const float r = std::max(1e-4f, obs.params.x);
            const float h = std::max(1e-4f, obs.params.y);
            return std::sqrt(r * r + 0.25f * h * h);
        }

        bool IntersectRaySphere(const glm::vec3& ray_o, const glm::vec3& ray_d, const glm::vec3& c, float r, float* out_t) {
            const glm::vec3 oc = ray_o - c;
            const float a      = glm::dot(ray_d, ray_d);
            const float b      = 2.0f * glm::dot(oc, ray_d);
            const float cc     = glm::dot(oc, oc) - r * r;
            const float disc   = b * b - 4.0f * a * cc;
            if (disc < 0.0f) {
                return false;
            }
            const float sqrt_disc = std::sqrt(disc);
            const float t0        = (-b - sqrt_disc) / (2.0f * a);
            const float t1        = (-b + sqrt_disc) / (2.0f * a);
            float t_hit           = -1.0f;
            if (t0 > 0.0f) {
                t_hit = t0;
            } else if (t1 > 0.0f) {
                t_hit = t1;
            }
            if (t_hit <= 0.0f) {
                return false;
            }
            if (out_t != nullptr) {
                *out_t = t_hit;
            }
            return true;
        }

    }  // namespace

    bool KinematicInputHandler::IsMouseInViewport(double x, double y, int viewport_w, int viewport_h) const {
        return (x >= 0.0 && x < static_cast<double>(viewport_w) && y >= 0.0 && y < static_cast<double>(viewport_h));
    }

    KinematicInputHandler::CameraInputResult KinematicInputHandler::UpdateCamera(omnilink::teleop_viewer::OrbitCamera* camera,
                                                                                 const UpdateContext& ctx) {
        CameraInputResult result;
        if (!camera) {
            return result;
        }

        if (first_mouse_) {
            prev_mouse_x_ = ctx.mouse_x;
            prev_mouse_y_ = ctx.mouse_y;
            first_mouse_  = false;
            return result;
        }

        double dx     = ctx.mouse_x - prev_mouse_x_;
        double dy     = ctx.mouse_y - prev_mouse_y_;
        prev_mouse_x_ = ctx.mouse_x;
        prev_mouse_y_ = ctx.mouse_y;

        bool mouse_in_viewport = IsMouseInViewport(ctx.mouse_x, ctx.mouse_y, ctx.viewport_w, ctx.viewport_h);
        bool block_camera      = ctx.panel_resize_active || ctx.ik_dragging_marker || ctx.ik_gizmo_using || ctx.ik_gizmo_over ||
                            ctx.obs_gizmo_using || ctx.obs_gizmo_over || ctx.imgui_wants_mouse;

        if (!mouse_in_viewport || block_camera) {
            return result;
        }

        if (ctx.left_mouse_down && !ctx.shift_key_down && !ctx.middle_mouse_down) {
            camera->rotate(static_cast<float>(dx), static_cast<float>(dy));
            result.consumed = true;
        } else if (ctx.middle_mouse_down || (ctx.shift_key_down && ctx.left_mouse_down)) {
            camera->pan(static_cast<float>(dx), static_cast<float>(dy));
            result.consumed = true;
        } else if (ctx.right_mouse_down) {
            camera->dolly(static_cast<float>(-dy));
            result.consumed = true;
        }
        if (std::fabs(ctx.scroll_delta) > 1e-6f) {
            camera->zoom(ctx.scroll_delta);
            result.consumed = true;
        }
        return result;
    }

    KinematicInputHandler::ObstaclePickResult KinematicInputHandler::UpdateObstaclePick(const UpdateContext& ctx, const glm::mat4& view,
                                                                                        const glm::mat4& proj,
                                                                                        const UserObstacleState& obstacles) {
        ObstaclePickResult result;
        bool mouse_in_viewport     = IsMouseInViewport(ctx.mouse_x, ctx.mouse_y, ctx.viewport_w, ctx.viewport_h);
        bool obstacle_pick_enabled = (ctx.sidebar_page == 0 || ctx.sidebar_page == 6);
        bool can_pick = obstacle_pick_enabled && mouse_in_viewport && !ctx.imgui_wants_mouse && !ctx.ik_gizmo_using && !ctx.ik_gizmo_over &&
                        !ctx.obs_gizmo_using && !ctx.obs_gizmo_over;
        if (!can_pick) {
            obstacle_pick_left_prev_ = ctx.left_mouse_down;
            return result;
        }

        bool left_clicked        = ctx.left_mouse_down && !obstacle_pick_left_prev_;
        obstacle_pick_left_prev_ = ctx.left_mouse_down;
        if (!left_clicked) {
            return result;
        }

        glm::vec3 ray_o(0.0f), ray_d(0.0f);
        if (!ComputeWorldRayFromScreen(static_cast<float>(ctx.mouse_x), static_cast<float>(ctx.mouse_y), ctx.viewport_w, ctx.viewport_h,
                                       view, proj, &ray_o, &ray_d)) {
            return result;
        }

        int best_index = -1;
        float best_t   = 1e9f;
        for (int i = 0; i < static_cast<int>(obstacles.items.size()); ++i) {
            const auto& obs = obstacles.items[static_cast<size_t>(i)];
            if (!obs.visible) {
                continue;
            }
            const float r = ObstaclePickRadius(obs);
            float hit_t   = 0.0f;
            if (IntersectRaySphere(ray_o, ray_d, obs.position, r, &hit_t) && hit_t < best_t) {
                best_t     = hit_t;
                best_index = i;
            }
        }
        if (best_index >= 0) {
            result.picked         = true;
            result.selected_index = best_index;
        }
        return result;
    }

    int KinematicInputHandler::HandleSidebarHotkeys(int current_page, bool enable_hotkeys) {
        if (!enable_hotkeys) {
            return current_page;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_1))
            return 0;
        if (ImGui::IsKeyPressed(ImGuiKey_2))
            return 1;
        if (ImGui::IsKeyPressed(ImGuiKey_3))
            return 2;
        if (ImGui::IsKeyPressed(ImGuiKey_4))
            return 3;
        if (ImGui::IsKeyPressed(ImGuiKey_5))
            return 4;
        if (ImGui::IsKeyPressed(ImGuiKey_6))
            return 5;
        if (ImGui::IsKeyPressed(ImGuiKey_7))
            return 6;
        return current_page;
    }

    void KinematicInputHandler::ResetMouseTracking() {
        first_mouse_ = true;
    }

}  // namespace kinematic_viewer
