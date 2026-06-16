#include "kinematic_viewer/kinematic_viewer_session.h"
#include "kinematic_viewer/kinematic_app.h"
#include "kinematic_viewer/kinematic_collision_monitor.h"
#include "kinematic_viewer/kinematic_ik_controller.h"
#include "kinematic_viewer/kinematic_initial_pose.h"
#include "kinematic_viewer/kinematic_line_renderer.h"
#include "kinematic_viewer/kinematic_marker_target_state.h"
#include "kinematic_viewer/kinematic_marker_utils.h"
#include "kinematic_viewer/kinematic_point_cloud.h"
#include "kinematic_viewer/kinematic_playback.h"
#include "kinematic_viewer/kinematic_playback_state_machine.h"
#include "kinematic_viewer/kinematic_render_loop.h"
#include "kinematic_viewer/kinematic_input_handler.h"
#include "kinematic_viewer/kinematic_link_kinematics.h"
#include "kinematic_viewer/kinematic_link_inspector.h"
#include "kinematic_viewer/kinematic_config_watcher.h"
#include "kinematic_viewer/kinematic_viewport_hud.h"
#include "kinematic_viewer/kinematic_angle_units.h"
#include "kinematic_viewer/kinematic_shader_utils.h"
#include "kinematic_viewer/kinematic_sidebar_layout.h"
#include "kinematic_viewer/kinematic_sidebar_panels.h"
#include "kinematic_viewer/kinematic_string_utils.h"
#include "kinematic_viewer/kinematic_ui_feedback.h"
#include "kinematic_viewer/kinematic_ui_theme.h"
#include "kinematic_viewer/kinematic_user_obstacles.h"
#include "kinematic_viewer/kinematic_video_panel.h"
#include "kinematic_viewer/kinematic_video_recorder.h"
#include "kinematic_viewer/kinematic_teach.h"
#include "kinematic_viewer/kinematic_teach_state.h"
#include "kinematic_viewer/kinematic_viewer_config.h"
#include "kinematic_viewer/rkv_panel_plugin.h"
#include "kinematic_viewer/rkv_panel_registry.h"
#include "rkv/scene.h"

#include "ImGuizmo.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <string>
#include <unistd.h>
#include <vector>

using kinematic_viewer::CollisionMonitor;
using kinematic_viewer::CollisionMonitorResult;
using kinematic_viewer::CollisionMonitorState;
using kinematic_viewer::createKinematicLineProgram;
using kinematic_viewer::createKinematicMeshProgram;
using kinematic_viewer::createKinematicPointProgram;
using kinematic_viewer::DebugPlaybackState;
using kinematic_viewer::DestroyUserObstacleGpuMeshes;
using kinematic_viewer::FocusCameraOnLink;
using kinematic_viewer::GetScrollDelta;
using kinematic_viewer::IkState;
using kinematic_viewer::InitialPoseApplyResult;
using kinematic_viewer::InitUserObstacleGpuMeshes;
using kinematic_viewer::KinematicApp;
using kinematic_viewer::KinematicIkController;
using kinematic_viewer::KinematicInputHandler;
using kinematic_viewer::KinematicLineRenderer;
using kinematic_viewer::KinematicPointCloudLayer;
using kinematic_viewer::KinematicRenderLoop;
using kinematic_viewer::KinematicUiFeedback;
using kinematic_viewer::KinematicViewerConfig;
using kinematic_viewer::LaunchConfig;
using kinematic_viewer::LinkKinematicsAnalyzer;
using kinematic_viewer::LoadTeachProgramFromYaml;
using kinematic_viewer::markerWorldMatrix;
using kinematic_viewer::MergeUserObstaclesIntoCollisionResult;
using kinematic_viewer::PointCloudColorModeFromString;
using kinematic_viewer::PointCloudLoadOptions;
using kinematic_viewer::PointCloudUiState;
using kinematic_viewer::RenderAngleUnitSelector;
using kinematic_viewer::RenderLinkInspectorPanel;
using kinematic_viewer::TeachProgramState;
using kinematic_viewer::TrajectoryPlayer;
using kinematic_viewer::UiSemanticLevel;
using kinematic_viewer::UserObstacleGpuMeshes;
using kinematic_viewer::ViewerState;
using kinematic_viewer::wrapDeltaDeg;
using rkv::OrbitCamera;
using rkv::RobotScene;

namespace kinematic_viewer {

    namespace session_internal {
        std::string ToLowerCopy(std::string text) {
            std::transform(text.begin(), text.end(), text.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
            return text;
        }
        bool IsMobileBaseDragRobot(const std::string& urdf_path, const std::vector<std::string>& keywords) {
            const std::string urdf_name_lc = ToLowerCopy(kinematic_viewer::PathBasename(urdf_path));
            for (const auto& keyword_raw : keywords) {
                const std::string keyword_lc = ToLowerCopy(keyword_raw);
                if (!keyword_lc.empty() && urdf_name_lc.find(keyword_lc) != std::string::npos) {
                    return true;
                }
            }
            return false;
        }
    }  // namespace session_internal

    struct KinematicViewerSession::FrameContext {
        double dt_sec                 = 0.0;
        double now_sec                = 0.0;
        double mouse_x                = 0.0;
        double mouse_y                = 0.0;
        int fb_w                      = 0;
        int fb_h                      = 0;
        int viewport_w                = 1;
        int viewport_h                = 1;
        int panel_w                   = 0;
        float panel_min               = 280.0f;
        float panel_max               = 0.0f;
        std::string current_panel_key = "scene";
        glm::mat4 view{1.0f};
        glm::mat4 proj{1.0f};
        glm::mat4 pick_view{1.0f};
        glm::mat4 pick_proj{1.0f};
        bool hover_for_joint_drag = false;
        bool was_playback_playing = false;
    };

    struct KinematicViewerSession::Impl {
        LaunchConfig launch;
        KinematicViewerConfig cfg;
        std::string urdf_path;
        KinematicApp* app  = nullptr;
        GLFWwindow* window = nullptr;
        int ui_theme_index = 0;
        ConfigWatcher config_watcher{""};

        GLuint mesh_shader  = 0;
        GLuint line_shader  = 0;
        GLuint point_shader = 0;
        KinematicLineRenderer line_renderer;
        KinematicRenderLoop render_loop;
        VideoRecorder video_recorder;
        UserObstacleGpuMeshes obstacle_meshes;
        RobotScene scene;
        OrbitCamera camera;
        ViewerState ui_state;
        PathPlannerUiState path_planner_ui;
        PointCloudUiState point_cloud_state;
        KinematicPointCloudLayer point_cloud_layer;
        RkvPanelRegistry panel_registry;
        IkState ik_state;
        DebugPlaybackState playback_state;
        PlaybackStateMachine playback_sm{nullptr};
        TeachProgramState teach_state;
        CollisionMonitorState collision_state;
        CollisionMonitorResult collision_result;
        TrajectoryPlayer trajectory_player;
        CollisionMonitor collision_monitor;
        LinkKinematicsAnalyzer link_kinematics_analyzer;
        KinematicUiFeedback ui_feedback;
        KinematicIkController ik_controller{nullptr};
        std::string last_playback_io_status;
        bool initial_pose_auto_apply_pending = false;
        int collision_refresh_tick           = 0;
        bool obstacle_gizmo_was_using        = false;
        bool obstacle_gizmo_was_over         = false;
        bool base_gizmo_was_using            = false;
        KinematicInputHandler input_handler;
        double last_frame_sec = 0.0;

        Impl() : playback_sm(&playback_state), ik_controller(&ik_state) {}

        void ShutdownGpu();
        void PersistConfigOnExit();
        InitialPoseApplyResult ApplyConfiguredInitialPose();
        void RunCollisionMonitor();

        void ApplyDeferredInitialPose(double now_sec);
        KinematicViewerSession::FrameContext BeginUiFrame(double mouse_x, double mouse_y);
        void HandleFrameHotkeys(KinematicViewerSession::FrameContext* frame);
        void UpdateViewportInput(KinematicViewerSession::FrameContext* frame, KinematicInputHandler::UpdateContext* input_ctx);
        void AdvanceSimulation(KinematicViewerSession::FrameContext* frame);
        void RenderViewport3D(const KinematicViewerSession::FrameContext& frame);
        void HandleImGuizmos(KinematicViewerSession::FrameContext* frame);
        void HandleViewportPickingAndCamera(KinematicViewerSession::FrameContext* frame, KinematicInputHandler::UpdateContext* input_ctx);
        void RenderViewportHud(const KinematicViewerSession::FrameContext& frame);
        void RenderSidebar(KinematicViewerSession::FrameContext* frame);
        void PresentFrame(KinematicApp& app, const KinematicViewerSession::FrameContext& frame);
    };

    InitialPoseApplyResult KinematicViewerSession::Impl::ApplyConfiguredInitialPose() {
        InitialPoseApplyResult result = kinematic_viewer::ApplyConfiguredInitialPose(cfg.initial_pose, &scene);
        ik_state.marker_initialized   = false;
        if (!ik_state.chains.empty()) {
            ik_controller.LoadActiveMarkerFromTarget(&scene);
        }
        return result;
    }

    void KinematicViewerSession::Impl::RunCollisionMonitor() {
        if (!collision_state.enable) {
            return;
        }
        collision_result = collision_monitor.Evaluate(collision_state, scene);
        MergeUserObstaclesIntoCollisionResult(ui_state.user_obstacles, scene, collision_state.warning_distance_m,
                                              collision_state.danger_distance_m, &collision_result);
        collision_monitor.UpdateStateFromResult(collision_result, &collision_state);
    }

    void KinematicViewerSession::Impl::ApplyDeferredInitialPose(double now_sec) {
        if (!initial_pose_auto_apply_pending) {
            return;
        }
        InitialPoseApplyResult result = ApplyConfiguredInitialPose();
        UiSemanticLevel level         = result.missing_joint_count > 0 ? UiSemanticLevel::Warning : UiSemanticLevel::Success;
        ui_feedback.Push(level, std::string("初始位姿加载: ") + result.detail, now_sec, 4.0);
        initial_pose_auto_apply_pending = false;
    }

    KinematicViewerSession::FrameContext KinematicViewerSession::Impl::BeginUiFrame(double mouse_x, double mouse_y) {
        KinematicViewerSession::FrameContext frame;
        frame.mouse_x = mouse_x;
        frame.mouse_y = mouse_y;

        glfwGetFramebufferSize(window, &frame.fb_w, &frame.fb_h);
        frame.panel_min = 280.0f;
        frame.panel_max = std::max(frame.panel_min, static_cast<float>(frame.fb_w) - 320.0f);
        ui_state.panel_width = std::clamp(ui_state.panel_width, frame.panel_min, frame.panel_max);
        frame.panel_w    = ui_state.sidebar_hidden ? 0 : static_cast<int>(ui_state.panel_width);
        frame.viewport_w = std::max(1, frame.fb_w - frame.panel_w);
        frame.viewport_h = std::max(1, frame.fb_h);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        ImGuizmo::BeginFrame();
        return frame;
    }

    void KinematicViewerSession::Impl::HandleFrameHotkeys(KinematicViewerSession::FrameContext* frame) {
        const bool sidebar_hotkeys_enabled = !ImGui::GetIO().WantTextInput && !ImGui::GetIO().WantCaptureKeyboard;
        ui_state.sidebar_page = input_handler.HandleSidebarHotkeys(ui_state.sidebar_page, panel_registry.Count(), sidebar_hotkeys_enabled);

        const auto viewport_hotkeys = input_handler.HandleViewportHotkeys(sidebar_hotkeys_enabled, playback_sm.HasKeyframes(),
                                                                          static_cast<float>(frame->dt_sec), playback_state.play_speed);
        if (viewport_hotkeys.toggled_sidebar) {
            ui_state.sidebar_hidden = !ui_state.sidebar_hidden;
            ui_feedback.Push(UiSemanticLevel::Info, ui_state.sidebar_hidden ? "已隐藏侧栏 (H 恢复)" : "已显示侧栏", frame->now_sec);
        }
        if (viewport_hotkeys.toggled_playback) {
            if (playback_sm.TogglePlayPause()) {
                ui_feedback.Push(UiSemanticLevel::Info, playback_sm.IsPlaying() ? "回放: 播放" : "回放: 暂停", frame->now_sec, 1.2);
            }
        }
        if (viewport_hotkeys.playback_speed_adjust != 0) {
            const float prev_speed    = playback_state.play_speed;
            playback_state.play_speed = std::clamp(playback_state.play_speed + viewport_hotkeys.playback_speed_adjust * 0.1f, 0.1f, 3.0f);
            if (playback_state.play_speed != prev_speed) {
                char speed_msg[64];
                std::snprintf(speed_msg, sizeof(speed_msg), "回放倍速: %.2fx", playback_state.play_speed);
                ui_feedback.Push(UiSemanticLevel::Info, speed_msg, frame->now_sec, 0.8);
            }
        }
        if (viewport_hotkeys.playback_step_count > 0 && viewport_hotkeys.playback_step_direction != 0) {
            for (int step = 0; step < viewport_hotkeys.playback_step_count; ++step) {
                if (playback_sm.StepKeyframe(viewport_hotkeys.playback_step_direction)) {
                    trajectory_player.SampleAtCurrentTime(playback_state, &scene);
                }
            }
        }
        if (panel_registry.Count() == 0) {
            ui_state.sidebar_page = 0;
        } else {
            ui_state.sidebar_page = std::clamp(ui_state.sidebar_page, 0, panel_registry.Count() - 1);
        }
        frame->current_panel_key    = panel_registry.Count() == 0 ? std::string("scene") : panel_registry.Id(ui_state.sidebar_page);
        ui_state.scene_panel_active = (frame->current_panel_key == "scene");
    }

    void KinematicViewerSession::Impl::UpdateViewportInput(KinematicViewerSession::FrameContext* frame,
                                                           KinematicInputHandler::UpdateContext* input_ctx) {
        input_ctx->mouse_x             = frame->mouse_x;
        input_ctx->mouse_y             = frame->mouse_y;
        input_ctx->viewport_w          = frame->viewport_w;
        input_ctx->viewport_h          = frame->viewport_h;
        input_ctx->imgui_wants_mouse   = ImGui::GetIO().WantCaptureMouse;
        input_ctx->panel_resize_active = ui_state.panel_resize_active;
        input_ctx->ik_gizmo_using      = ik_state.gizmo_was_using;
        input_ctx->ik_gizmo_over       = ik_state.gizmo_was_over;
        input_ctx->obs_gizmo_using     = obstacle_gizmo_was_using || base_gizmo_was_using;
        input_ctx->obs_gizmo_over      = obstacle_gizmo_was_over;
        input_ctx->ik_dragging_marker  = ik_state.dragging_marker;
        input_ctx->sidebar_page        = (frame->current_panel_key == "scene") ? 0 : ((frame->current_panel_key == "obstacle") ? 6 : -1);
        input_ctx->scroll_delta        = GetScrollDelta();
        input_ctx->left_mouse_down     = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
        input_ctx->middle_mouse_down   = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
        input_ctx->right_mouse_down    = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
        input_ctx->shift_key_down =
            glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
        input_ctx->enable_joint_drag = ui_state.enable_joint_drag_rotation;

        frame->pick_proj = glm::perspective(glm::radians(50.0f),
                                            static_cast<float>(frame->viewport_w) / static_cast<float>(frame->viewport_h), 0.05f, 80.0f);
        frame->pick_view = camera.viewMatrix();

        frame->hover_for_joint_drag = ui_state.enable_joint_drag_rotation && !input_ctx->ik_gizmo_using && !input_ctx->obs_gizmo_using &&
                                      !input_ctx->imgui_wants_mouse;
        if (frame->hover_for_joint_drag || ui_state.enable_link_hover_highlight) {
            auto link_hover = input_handler.UpdateLinkHover(*input_ctx, frame->pick_view, frame->pick_proj, &scene, glfwGetTime(),
                                                            frame->hover_for_joint_drag);
            if (!link_hover.throttle_skip) {
                ui_state.hovered_link = link_hover.picked ? link_hover.link_name : std::string();
            }
        } else if (!input_handler.IsJointDragActive()) {
            ui_state.hovered_link.clear();
        }

        input_ctx->hovered_link = &ui_state.hovered_link;
        if (!ui_state.hovered_link.empty()) {
            std::string parent_joint;
            input_ctx->hovered_link_draggable = scene.getParentJointNameForLink(ui_state.hovered_link, &parent_joint) && [&]() {
                rkv::RobotScene::JointInfo joint_info;
                return scene.getJointInfo(parent_joint, &joint_info) && joint_info.revolute;
            }();
        } else {
            input_ctx->hovered_link_draggable = false;
        }

        auto joint_drag = input_handler.UpdateJointDrag(*input_ctx, frame->pick_view, frame->pick_proj, &scene);
        if (joint_drag.started) {
            ui_state.selected_link            = joint_drag.link_name;
            ui_state.trajectory_min_surface_m = -1.0f;
            const auto joints                 = scene.getJointInfos();
            ui_state.selected_joint           = -1;
            for (int i = 0; i < static_cast<int>(joints.size()); ++i) {
                if (joints[static_cast<size_t>(i)].name == joint_drag.joint_name) {
                    ui_state.selected_joint = i;
                    break;
                }
            }
        } else if (joint_drag.dragging) {
            ui_state.selected_link = joint_drag.link_name;
        }

        if (scene.isJointPoseDirty()) {
            scene.updateTransforms();
        }
    }

    void KinematicViewerSession::Impl::AdvanceSimulation(KinematicViewerSession::FrameContext* frame) {
        frame->was_playback_playing = playback_sm.IsPlaying();
        playback_sm.AdvanceTime(static_cast<float>(frame->dt_sec));
        if (playback_sm.IsPlaying()) {
            trajectory_player.SampleAtCurrentTime(playback_state, &scene);
        }
        scene.setFixedBaseMode(ui_state.lock_base);
        scene.updateTransforms();

        if (collision_state.enable && !input_handler.IsJointDragActive() &&
            (scene.isJointPoseDirty() || playback_sm.IsPlaying() || (++collision_refresh_tick % 4 == 0))) {
            RunCollisionMonitor();
        }

        frame->proj = glm::perspective(glm::radians(50.0f), static_cast<float>(frame->viewport_w) / static_cast<float>(frame->viewport_h),
                                       0.05f, 80.0f);
        frame->view = camera.viewMatrix();
    }

    void KinematicViewerSession::Impl::RenderViewport3D(const KinematicViewerSession::FrameContext& frame) {
        KinematicRenderLoop::Context render_ctx;
        render_ctx.viewport_w        = frame.viewport_w;
        render_ctx.viewport_h        = frame.viewport_h;
        render_ctx.mesh_shader       = mesh_shader;
        render_ctx.line_shader       = line_shader;
        render_ctx.point_shader      = point_shader;
        render_ctx.point_cloud       = &point_cloud_state;
        render_ctx.point_cloud_layer = &point_cloud_layer;
        render_ctx.scene             = &scene;
        render_ctx.ui_state          = &ui_state;
        render_ctx.ik_state          = &ik_state;
        render_ctx.collision_state   = &collision_state;
        render_ctx.collision_result  = &collision_result;
        render_ctx.obstacle_meshes   = &obstacle_meshes;
        render_ctx.camera            = &camera;
        render_ctx.planned_path      = &path_planner_ui.preview_waypoints;
        render_ctx.show_planned_path = path_planner_ui.show_preview;
        render_ctx.demo_visual_mode  = ui_state.demo_visual_mode;
        render_loop.Render(render_ctx);
        kinematic_viewer::CaptureFrameForRecorder(&video_recorder, frame.viewport_w, frame.viewport_h);
    }

    void KinematicViewerSession::Impl::HandleImGuizmos(KinematicViewerSession::FrameContext* frame) {
        ik_controller.ApplyExternalTarget(&scene);

        auto applyIkForActiveChain = [&](bool force_orientation_lock, bool fast_mode, bool prefer_position_only_target) -> bool {
            return ik_controller.ApplyIkForActiveChain(&scene, force_orientation_lock, fast_mode, prefer_position_only_target);
        };
        auto refineActiveChainToMarker = [&]() -> bool {
            return ik_controller.RefineActiveChainToMarker(&scene);
        };
        auto activeChainPositionErrorMmToMarker = [&]() -> float {
            return ik_controller.ActiveChainPositionErrorMmToMarker(&scene);
        };

        ImGuizmo::SetOrthographic(false);
        ImGuizmo::AllowAxisFlip(false);
        ImGuizmo::SetDrawlist(ImGui::GetForegroundDrawList());
        ImGuizmo::SetRect(0.0f, 0.0f, static_cast<float>(frame->viewport_w), static_cast<float>(frame->viewport_h));

        const bool obstacle_page_active = (frame->current_panel_key == "scene" || frame->current_panel_key == "obstacle");
        const bool obstacle_edit_active = obstacle_page_active && ui_state.user_obstacles.enable_pose_gizmo &&
                                          ui_state.user_obstacles.selected_index >= 0 &&
                                          ui_state.user_obstacles.selected_index < static_cast<int>(ui_state.user_obstacles.items.size());
        if (obstacle_edit_active) {
            auto& obs = ui_state.user_obstacles.items[static_cast<size_t>(ui_state.user_obstacles.selected_index)];
            ImGuizmo::SetGizmoSizeClipSpace(ui_state.user_obstacles.gizmo_size_clip_space);
            glm::mat4 obs_world        = markerWorldMatrix(obs.position, obs.rpy_deg);
            ImGuizmo::OPERATION obs_op = static_cast<ImGuizmo::OPERATION>(ImGuizmo::TRANSLATE | ImGuizmo::ROTATE);
            if (ui_state.user_obstacles.gizmo_operation == 0) {
                obs_op = ImGuizmo::TRANSLATE;
            } else if (ui_state.user_obstacles.gizmo_operation == 1) {
                obs_op = ImGuizmo::ROTATE;
            }
            ImGuizmo::MODE obs_mode = ui_state.user_obstacles.gizmo_world_mode ? ImGuizmo::WORLD : ImGuizmo::LOCAL;
            glm::mat4 obs_delta(1.0f);
            bool obs_manipulated = ImGuizmo::Manipulate(glm::value_ptr(frame->view), glm::value_ptr(frame->proj), obs_op, obs_mode,
                                                        glm::value_ptr(obs_world), glm::value_ptr(obs_delta), nullptr);
            bool obs_using       = ImGuizmo::IsUsing();
            bool obs_over        = ImGuizmo::IsOver();
            if (obs_manipulated || obs_using) {
                const glm::vec3 raw_pos = glm::vec3(obs_world[3]);
                const glm::quat raw_q   = glm::quat_cast(obs_world);
                const glm::vec3 raw_rpy_deg(glm::degrees(glm::eulerAngles(raw_q)));
                obs.position = raw_pos;
                obs.rpy_deg  = raw_rpy_deg;
            }
            obstacle_gizmo_was_using = obs_using;
            obstacle_gizmo_was_over  = obs_over;
        } else {
            obstacle_gizmo_was_using = false;
            obstacle_gizmo_was_over  = false;
        }

        const bool base_edit_active =
            ui_state.mobile_base_drag_available && ui_state.mobile_base_drag_enabled && frame->current_panel_key == "scene";
        if (base_edit_active && !obstacle_edit_active && !ik_state.gizmo_was_using) {
            float base_x_m = 0.0f;
            float base_y_m = 0.0f;
            float base_yaw = 0.0f;
            if (scene.getVirtualBasePose2D(&base_x_m, &base_y_m, &base_yaw)) {
                ImGuizmo::SetGizmoSizeClipSpace(0.12f);
                glm::mat4 base_world =
                    markerWorldMatrix(glm::vec3(base_x_m, base_y_m, 0.0f), glm::vec3(0.0f, 0.0f, glm::degrees(base_yaw)));
                ImGuizmo::OPERATION base_op = static_cast<ImGuizmo::OPERATION>(ImGuizmo::TRANSLATE | ImGuizmo::ROTATE);
                if (ui_state.mobile_base_gizmo_operation == 0) {
                    base_op = ImGuizmo::TRANSLATE;
                } else if (ui_state.mobile_base_gizmo_operation == 1) {
                    base_op = ImGuizmo::ROTATE;
                }
                glm::mat4 base_delta(1.0f);
                bool base_manipulated =
                    ImGuizmo::Manipulate(glm::value_ptr(frame->view), glm::value_ptr(frame->proj), base_op, ImGuizmo::WORLD,
                                         glm::value_ptr(base_world), glm::value_ptr(base_delta), nullptr);
                bool base_using = ImGuizmo::IsUsing();
                if (base_manipulated || base_using) {
                    const glm::vec3 raw_pos = glm::vec3(base_world[3]);
                    const glm::mat3 rot_mat(base_world);
                    // Planar yaw about world Z (do not use glm::eulerAngles — unstable for gizmo output).
                    const float yaw_rad = std::atan2(rot_mat[0][1], rot_mat[0][0]);
                    float new_x         = raw_pos.x;
                    float new_y         = raw_pos.y;
                    float new_yaw       = yaw_rad;
                    if (ui_state.mobile_base_gizmo_operation == 0) {
                        new_yaw = base_yaw;
                    } else if (ui_state.mobile_base_gizmo_operation == 1) {
                        new_x = base_x_m;
                        new_y = base_y_m;
                    }
                    scene.setVirtualBasePose2D(new_x, new_y, new_yaw);
                }
                base_gizmo_was_using = base_using;
            } else {
                base_gizmo_was_using = false;
            }
        } else {
            base_gizmo_was_using = false;
        }

        ImGuizmo::SetGizmoSizeClipSpace(ik_state.gizmo_size_clip_space);
        const bool ik_page_active = (frame->current_panel_key == "ik");
        if (ik_page_active && !obstacle_edit_active && !base_gizmo_was_using && ik_state.selected_chain >= 0 &&
            ik_state.selected_chain < static_cast<int>(ik_state.chains.size())) {
            if (!ik_state.marker_initialized) {
                ik_controller.LoadActiveMarkerFromTarget(&scene);
            }

            const glm::vec3 marker_pos(ik_state.marker_pos[0], ik_state.marker_pos[1], ik_state.marker_pos[2]);
            const glm::vec3 marker_rpy_deg(ik_state.marker_rpy_deg[0], ik_state.marker_rpy_deg[1], ik_state.marker_rpy_deg[2]);
            const glm::mat4 marker_world_before = markerWorldMatrix(marker_pos, marker_rpy_deg);
            glm::mat4 gizmo_world               = marker_world_before;
            ImGuizmo::OPERATION op              = static_cast<ImGuizmo::OPERATION>(ImGuizmo::TRANSLATE | ImGuizmo::ROTATE);
            if (ik_state.gizmo_operation == 0)
                op = ImGuizmo::TRANSLATE;
            if (ik_state.gizmo_operation == 1)
                op = ImGuizmo::ROTATE;
            ImGuizmo::MODE mode = ik_state.gizmo_world_mode ? ImGuizmo::WORLD : ImGuizmo::LOCAL;

            float snap_values[3] = {0.0f, 0.0f, 0.0f};
            float* snap_ptr      = nullptr;
            if (op == ImGuizmo::TRANSLATE && ik_state.translate_snap_enabled) {
                snap_values[0] = ik_state.translate_snap_step_m;
                snap_values[1] = ik_state.translate_snap_step_m;
                snap_values[2] = ik_state.translate_snap_step_m;
                snap_ptr       = snap_values;
            } else if (op == ImGuizmo::ROTATE && ik_state.rotate_snap_enabled) {
                snap_values[0] = ik_state.rotate_snap_step_deg;
                snap_values[1] = ik_state.rotate_snap_step_deg;
                snap_values[2] = ik_state.rotate_snap_step_deg;
                snap_ptr       = snap_values;
            }

            glm::mat4 gizmo_delta(1.0f);
            bool manipulated                = ImGuizmo::Manipulate(glm::value_ptr(frame->view), glm::value_ptr(frame->proj), op, mode,
                                                                   glm::value_ptr(gizmo_world), glm::value_ptr(gizmo_delta), snap_ptr);
            bool gizmo_using                = ImGuizmo::IsUsing();
            bool gizmo_over                 = ImGuizmo::IsOver();
            bool current_drag_position_only = (op == ImGuizmo::TRANSLATE);
            if (manipulated || gizmo_using) {
                ik_state.dragging_marker       = true;
                ik_state.gizmo_drag_interacted = true;

                const glm::vec3 raw_pos = glm::vec3(gizmo_world[3]);
                const glm::quat raw_q   = glm::quat_cast(gizmo_world);
                const glm::vec3 raw_rpy_deg(glm::degrees(glm::eulerAngles(raw_q)));

                const glm::vec3 delta_pos = raw_pos - marker_pos;
                glm::vec3 scaled_delta_pos(delta_pos.x * ik_state.translate_channel_gain[0],
                                           delta_pos.y * ik_state.translate_channel_gain[1],
                                           delta_pos.z * ik_state.translate_channel_gain[2]);

                const glm::vec3 raw_delta_rpy_deg(wrapDeltaDeg(raw_rpy_deg.x - marker_rpy_deg.x),
                                                  wrapDeltaDeg(raw_rpy_deg.y - marker_rpy_deg.y),
                                                  wrapDeltaDeg(raw_rpy_deg.z - marker_rpy_deg.z));
                glm::vec3 scaled_delta_rpy_deg(raw_delta_rpy_deg.x * ik_state.rotate_channel_gain[0],
                                               raw_delta_rpy_deg.y * ik_state.rotate_channel_gain[1],
                                               raw_delta_rpy_deg.z * ik_state.rotate_channel_gain[2]);

                const glm::vec3 updated_pos = marker_pos + scaled_delta_pos;
                const glm::vec3 updated_rpy_deg(marker_rpy_deg.x + scaled_delta_rpy_deg.x, marker_rpy_deg.y + scaled_delta_rpy_deg.y,
                                                marker_rpy_deg.z + scaled_delta_rpy_deg.z);

                current_drag_position_only =
                    glm::length(scaled_delta_pos) > 1e-6f && glm::length(glm::radians(scaled_delta_rpy_deg)) < glm::radians(0.25f);
                if (op == ImGuizmo::TRANSLATE) {
                    current_drag_position_only = true;
                } else if (op == ImGuizmo::ROTATE) {
                    current_drag_position_only = false;
                }
                ik_state.gizmo_drag_position_only = current_drag_position_only;

                ik_state.marker_pos[0] = updated_pos.x;
                ik_state.marker_pos[1] = updated_pos.y;
                ik_state.marker_pos[2] = updated_pos.z;
                if (!ik_state.lock_orientation) {
                    ik_state.marker_rpy_deg[0] = updated_rpy_deg.x;
                    ik_state.marker_rpy_deg[1] = updated_rpy_deg.y;
                    ik_state.marker_rpy_deg[2] = updated_rpy_deg.z;
                }
                ik_controller.SaveActiveMarkerToTarget();
                ik_state.gizmo_pose_dirty = true;
            } else {
                ik_state.dragging_marker = false;
            }

            // Realtime IK updates while dragging (RViz-like immediate feedback), throttled by frequency.
            if (gizmo_using && ik_state.gizmo_pose_dirty && ik_state.realtime_ik_during_drag) {
                const float effective_hz  = std::max(5.0f, ik_state.realtime_ik_hz);
                const double interval_sec = 1.0 / static_cast<double>(effective_hz);
                if (ik_state.last_realtime_ik_apply_sec < 0.0 || (frame->now_sec - ik_state.last_realtime_ik_apply_sec) >= interval_sec) {
                    applyIkForActiveChain(false, true, current_drag_position_only);
                    ik_state.last_realtime_ik_apply_sec = frame->now_sec;
                    ik_state.gizmo_pose_dirty           = false;
                }
            }

            // Sync IK only when drag ends (mouse release / gizmo released)
            if (!gizmo_using && ik_state.gizmo_was_using && ik_state.gizmo_drag_interacted) {
                applyIkForActiveChain(false, false, ik_state.gizmo_drag_position_only);
                const bool drag_had_rotation               = !ik_state.gizmo_drag_position_only;
                const bool should_refine_with_single_chain = ik_state.solve_mode == "full_body" && ui_state.lock_base &&
                                                             ik_state.refine_single_chain_on_drag_end &&
                                                             (!ik_state.refine_only_when_rotation || drag_had_rotation);
                if (should_refine_with_single_chain) {
                    // Strongly close tip-to-marker gap after full-body drag end.
                    for (int pass = 0; pass < 3; ++pass) {
                        const float err_mm = activeChainPositionErrorMmToMarker();
                        if (err_mm <= 1.0f) {
                            break;
                        }
                        if (!refineActiveChainToMarker()) {
                            break;
                        }
                    }
                }
                ik_state.gizmo_pose_dirty         = false;
                ik_state.gizmo_drag_interacted    = false;
                ik_state.gizmo_drag_position_only = true;
            }
            ik_state.gizmo_was_using = gizmo_using;
            ik_state.gizmo_was_over  = gizmo_over;
        } else {
            ik_state.gizmo_was_using = false;
            ik_state.gizmo_was_over  = false;
        }
    }

    void KinematicViewerSession::Impl::HandleViewportPickingAndCamera(KinematicViewerSession::FrameContext* frame,
                                                                      KinematicInputHandler::UpdateContext* input_ctx) {
        auto obstacle_pick = input_handler.UpdateObstaclePick(*input_ctx, frame->pick_view, frame->pick_proj, ui_state.user_obstacles);
        if (obstacle_pick.picked) {
            ui_state.user_obstacles.selected_index = obstacle_pick.selected_index;
        }

        if (!frame->hover_for_joint_drag && ui_state.enable_link_hover_highlight) {
            auto link_hover = input_handler.UpdateLinkHover(*input_ctx, frame->pick_view, frame->pick_proj, &scene, glfwGetTime(), false);
            if (!link_hover.throttle_skip) {
                ui_state.hovered_link = link_hover.picked ? link_hover.link_name : std::string();
            }
        } else if (!ui_state.enable_link_hover_highlight && !ui_state.enable_joint_drag_rotation) {
            ui_state.hovered_link.clear();
        }

        if (ui_state.enable_link_click_select) {
            auto link_pick = input_handler.UpdateLinkPick(*input_ctx, frame->pick_view, frame->pick_proj, &scene);
            if (link_pick.picked) {
                ui_state.selected_link            = link_pick.link_name;
                ui_state.trajectory_min_surface_m = -1.0f;
                ui_state.selected_joint           = -1;
            }
        }

        input_ctx->ik_gizmo_using         = ik_state.gizmo_was_using;
        input_ctx->ik_dragging_marker     = ik_state.dragging_marker;
        input_ctx->obs_gizmo_using        = obstacle_gizmo_was_using || base_gizmo_was_using;
        input_ctx->joint_drag_active      = input_handler.IsJointDragActive();
        input_ctx->hovered_link_draggable = input_ctx->hovered_link_draggable || input_handler.IsJointDragActive();
        input_ctx->imgui_wants_mouse      = ImGui::GetIO().WantCaptureMouse;
        input_handler.UpdateCamera(&camera, *input_ctx);
    }

    void KinematicViewerSession::Impl::RenderViewportHud(const KinematicViewerSession::FrameContext& frame) {
        // --- Viewport HUD (playback bar, overlays) ---
        kinematic_viewer::ViewportHudContext hud_ctx;
        hud_ctx.viewport_w       = frame.viewport_w;
        hud_ctx.viewport_h       = frame.viewport_h;
        hud_ctx.ui_state         = &ui_state;
        hud_ctx.playback_state   = &playback_state;
        hud_ctx.playback_sm      = &playback_sm;
        hud_ctx.playback_player  = &trajectory_player;
        hud_ctx.scene            = &scene;
        hud_ctx.collision_state  = &collision_state;
        hud_ctx.collision_result = &collision_result;
        hud_ctx.video_recorder   = &video_recorder;
        kinematic_viewer::RenderViewportHud(hud_ctx);
    }

    void KinematicViewerSession::Impl::RenderSidebar(KinematicViewerSession::FrameContext* frame) {
        // --- Sidebar panels ---
        if (frame->panel_w > 0) {
            glViewport(frame->viewport_w, 0, frame->panel_w, frame->fb_h);
            glDisable(GL_DEPTH_TEST);
            ImGui::SetNextWindowPos(ImVec2(static_cast<float>(frame->viewport_w), 0.0f), ImGuiCond_Always);
            ImGui::SetNextWindowSize(ImVec2(static_cast<float>(frame->panel_w), static_cast<float>(frame->fb_h)), ImGuiCond_Always);
            ImGui::Begin("机器人运动学调试", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove);

            {
                ImGuiIO& io            = ImGui::GetIO();
                const float grip_width = 8.0f;
                ImVec2 window_pos      = ImGui::GetWindowPos();
                ImVec2 window_size     = ImGui::GetWindowSize();
                ImVec2 grip_min(window_pos.x, window_pos.y);
                ImVec2 grip_max(window_pos.x + grip_width, window_pos.y + window_size.y);
                ImGui::SetCursorScreenPos(grip_min);
                ImGui::InvisibleButton("##sidebar_resize_grip_kin", ImVec2(grip_width, window_size.y));
                bool grip_hovered            = ImGui::IsItemHovered();
                bool grip_active             = ImGui::IsItemActive();
                ui_state.panel_resize_active = grip_active;
                if (grip_hovered || grip_active) {
                    ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
                }
                if (grip_active) {
                    ui_state.panel_width = std::clamp(ui_state.panel_width - io.MouseDelta.x, frame->panel_min, frame->panel_max);
                }
                ImU32 grip_color = grip_active ? IM_COL32(120, 200, 255, 220)
                                               : (grip_hovered ? IM_COL32(120, 180, 240, 180) : IM_COL32(80, 110, 150, 120));
                ImGui::GetWindowDrawList()->AddRectFilled(grip_min, grip_max, grip_color, 2.0f);
                ImGui::SetCursorPosY(8.0f);
            }

            {
                const std::string urdf_name = kinematic_viewer::PathBasename(urdf_path);
                ImGui::Text("URDF: %s", urdf_name.c_str());
                if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
                    ImGui::SetTooltip("%s", urdf_path.c_str());
                }
            }
            if (ImGui::CollapsingHeader("快捷操作")) {
                if (ImGui::SmallButton("重置视角")) {
                    camera.distance = cfg.camera.distance;
                    camera.yaw      = cfg.camera.yaw;
                    camera.pitch    = cfg.camera.pitch;
                    camera.target   = cfg.camera.target;
                }
                ImGui::SameLine();
                if (ImGui::SmallButton("对准Marker")) {
                    camera.target = glm::vec3(ik_state.marker_pos[0], ik_state.marker_pos[1], ik_state.marker_pos[2]);
                }
                ImGui::SameLine();
                bool has_selected_obstacle =
                    ui_state.user_obstacles.selected_index >= 0 &&
                    ui_state.user_obstacles.selected_index < static_cast<int>(ui_state.user_obstacles.items.size());
                if (!has_selected_obstacle) {
                    ImGui::BeginDisabled();
                }
                if (ImGui::SmallButton("对准障碍")) {
                    const auto& obs = ui_state.user_obstacles.items[static_cast<size_t>(ui_state.user_obstacles.selected_index)];
                    camera.target   = obs.position;
                }
                if (!has_selected_obstacle) {
                    ImGui::EndDisabled();
                }
                ImGui::SameLine();
                const bool has_selected_link = !ui_state.selected_link.empty();
                if (!has_selected_link) {
                    ImGui::BeginDisabled();
                }
                if (ImGui::SmallButton("对准Link")) {
                    FocusCameraOnLink(&camera, scene, ui_state.selected_link);
                }
                if (!has_selected_link) {
                    ImGui::EndDisabled();
                }
            }

            if (ImGui::CollapsingHeader("视频录制")) {
                kinematic_viewer::RenderVideoRecorderPanel(&ui_state, &video_recorder, &ui_feedback, frame->now_sec, frame->viewport_w,
                                                           frame->viewport_h);
            }
            if (cfg.initial_pose.enable) {
                if (ImGui::Button("加载初始位姿")) {
                    InitialPoseApplyResult result = ApplyConfiguredInitialPose();
                    UiSemanticLevel level         = result.missing_joint_count > 0 ? UiSemanticLevel::Warning : UiSemanticLevel::Success;
                    ui_feedback.Push(level, std::string("初始位姿加载: ") + result.detail, frame->now_sec, 4.0);
                }
                ImGui::SameLine();
                ImGui::TextDisabled("来自 config.initial_pose");
            }
            {
                const auto& theme_names = kinematic_viewer::KinematicUiThemeNames();
                int new_theme_index     = ui_theme_index;
                kinematic_viewer::PushSidebarFullWidth();
                if (ImGui::Combo("主题", &new_theme_index, theme_names.data(), static_cast<int>(theme_names.size()))) {
                    ui_theme_index = new_theme_index;
                    kinematic_viewer::ApplyKinematicUiStyleByIndex(ui_theme_index);
                    ui_feedback.Push(UiSemanticLevel::Info, std::string("主题切换: ") + theme_names[static_cast<size_t>(ui_theme_index)],
                                     frame->now_sec);
                }
                kinematic_viewer::PopSidebarWidth();
            }
            {
                auto playbackLevel        = UiSemanticLevel::Info;
                const char* playbackLabel = "回放 STOP";
                if (playback_sm.IsPlaying()) {
                    playbackLevel = UiSemanticLevel::Success;
                    playbackLabel = "回放 PLAY";
                } else if (playback_sm.IsPaused()) {
                    playbackLevel = UiSemanticLevel::Warning;
                    playbackLabel = "回放 PAUSE";
                }

                auto collisionLevel = UiSemanticLevel::Info;
                std::string collisionLabel("碰撞 --");
                if (collision_state.has_valid_distance) {
                    if (collision_state.nearest_surface_distance_m <= collision_state.danger_distance_m) {
                        collisionLevel = UiSemanticLevel::Error;
                        collisionLabel = "碰撞 DANGER";
                    } else if (collision_state.nearest_surface_distance_m <= collision_state.warning_distance_m) {
                        collisionLevel = UiSemanticLevel::Warning;
                        collisionLabel = "碰撞 WARN";
                    } else {
                        collisionLevel = UiSemanticLevel::Success;
                        collisionLabel = "碰撞 SAFE";
                    }
                }

                ImGui::BeginChild("##top_status_chips", ImVec2(0.0f, 34.0f), false,
                                  ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
                KinematicUiFeedback::RenderStatusChip(ik_state.solve_mode == "full_body" ? "IK FULL_BODY" : "IK SINGLE",
                                                      UiSemanticLevel::Info);
                ImGui::SameLine();
                KinematicUiFeedback::RenderStatusChip(playbackLabel, playbackLevel);
                ImGui::SameLine();
                KinematicUiFeedback::RenderStatusChip(collisionLabel.c_str(), collisionLevel);
                ImGui::EndChild();
            }
            if (ImGui::CollapsingHeader("操作提示")) {
                ImGui::TextDisabled("视角：左键旋转，中键/Shift+左键平移，右键缩放，滚轮缩放");
                ImGui::TextDisabled("Space 播放/暂停  ·  A/D 逐帧  ·  W/S 加减速  ·  H 隐藏/显示侧栏  ·  1-9 切换子页");
                ImGui::TextDisabled("场景页可开启 3D 点选 Link；演示视觉适合录屏展示");
            }
            ImGui::Separator();
            float avail_w = ImGui::GetContentRegionAvail().x;
            float used_w  = 0.0f;
            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(5.0f, 3.0f));
            for (int i = 0; i < panel_registry.Count(); ++i) {
                if (i > 0) {
                    const float spacing = ImGui::GetStyle().ItemSpacing.x;
                    const float btn_w   = ImGui::CalcTextSize(panel_registry.Label(i).c_str()).x + ImGui::GetStyle().FramePadding.x * 2.0f;
                    if (used_w + spacing + btn_w <= avail_w) {
                        ImGui::SameLine();
                        used_w += spacing;
                    } else {
                        used_w = 0.0f;
                    }
                }
                const bool selected = (ui_state.sidebar_page == i);
                if (selected) {
                    ImGui::PushStyleColor(ImGuiCol_Button, IM_COL32(90, 155, 235, 255));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, IM_COL32(100, 168, 252, 255));
                    ImGui::PushStyleColor(ImGuiCol_ButtonActive, IM_COL32(70, 132, 208, 255));
                }
                if (ImGui::Button(panel_registry.Label(i).c_str())) {
                    ui_state.sidebar_page = i;
                }
                used_w += ImGui::GetItemRectSize().x;
                if (selected) {
                    ImGui::PopStyleColor(3);
                }
            }
            ImGui::PopStyleVar();
            ImGui::Separator();

            kinematic_viewer::BeginSidebarScrollRegion("##sidebar_scroll");
            kinematic_viewer::RenderAngleUnitSelector(&ui_state.angle_unit_deg);
            ImGui::Separator();

            // link inspector is shared between scene and tf panels.
            if (frame->current_panel_key == "scene" || frame->current_panel_key == "tf") {
                RenderLinkInspectorPanel(&ui_state, &scene, &camera, &collision_state, &collision_result, &playback_state,
                                         &collision_monitor, &link_kinematics_analyzer);
            }

            auto joints   = scene.getJointInfos();
            auto tf_infos = scene.getLinkTfInfos();
            kinematic_viewer::TickTrajectorySequence(&playback_state, &playback_sm, frame->was_playback_playing, joints, &trajectory_player,
                                                     &scene);

            // Build the context bag and dispatch to the active panel plugin.
            RkvPanelCtx panel_ctx{};
            panel_ctx.viewer_state             = &ui_state;
            panel_ctx.ik_state                 = &ik_state;
            panel_ctx.ik_controller            = &ik_controller;
            panel_ctx.scene                    = &scene;
            panel_ctx.camera                   = &camera;
            panel_ctx.collision_state          = &collision_state;
            panel_ctx.collision_result         = &collision_result;
            panel_ctx.collision_monitor        = &collision_monitor;
            panel_ctx.playback_state           = &playback_state;
            panel_ctx.playback_player          = &trajectory_player;
            panel_ctx.playback_sm              = &playback_sm;
            panel_ctx.teach_state              = &teach_state;
            panel_ctx.point_cloud_state        = &point_cloud_state;
            panel_ctx.point_cloud_layer        = &point_cloud_layer;
            panel_ctx.path_planner_ui          = &path_planner_ui;
            panel_ctx.link_kinematics_analyzer = &link_kinematics_analyzer;
            panel_ctx.joints                   = &joints;
            panel_ctx.tf_infos                 = &tf_infos;
            panel_ctx.ik_solver                = &ik_state.solver;
            panel_ctx.ik_chains                = &ik_state.chains;

            panel_registry.Render(ui_state.sidebar_page, &panel_ctx);

            kinematic_viewer::EndSidebarScrollRegion();

            if (scene.consumeJointPoseDirty()) {
                scene.updateTransforms();
                RunCollisionMonitor();
            }

            if (playback_state.trajectory_io_status != last_playback_io_status) {
                if (!playback_state.trajectory_io_status.empty()) {
                    UiSemanticLevel level = UiSemanticLevel::Info;
                    if (playback_state.trajectory_io_status.find("失败") != std::string::npos) {
                        level = UiSemanticLevel::Error;
                    } else if (playback_state.trajectory_io_status.find("成功") != std::string::npos) {
                        level = UiSemanticLevel::Success;
                    } else if (playback_state.trajectory_io_status.find("告警") != std::string::npos) {
                        level = UiSemanticLevel::Warning;
                    }
                    ui_feedback.Push(level, playback_state.trajectory_io_status, frame->now_sec,
                                     level == UiSemanticLevel::Error ? 4.5 : 2.8);
                }
                last_playback_io_status = playback_state.trajectory_io_status;
            }

            ImGui::End();
        }

        ui_feedback.RenderToasts(frame->now_sec, static_cast<float>(frame->viewport_w) - 14.0f, 14.0f);
    }

    void KinematicViewerSession::Impl::PresentFrame(KinematicApp& app, const KinematicViewerSession::FrameContext& frame) {
        // --- Present ---
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        app.SwapBuffers();
    }

    KinematicViewerSession::KinematicViewerSession() : impl_(new Impl()) {}

    KinematicViewerSession::~KinematicViewerSession() {
        if (impl_ != nullptr) {
            impl_->ShutdownGpu();
            delete impl_;
            impl_ = nullptr;
        }
    }

    void KinematicViewerSession::Impl::ShutdownGpu() {
        DestroyUserObstacleGpuMeshes(&obstacle_meshes);
        if (mesh_shader != 0) {
            glDeleteProgram(mesh_shader);
            mesh_shader = 0;
        }
        if (line_shader != 0) {
            glDeleteProgram(line_shader);
            line_shader = 0;
        }
        if (point_shader != 0) {
            glDeleteProgram(point_shader);
            point_shader = 0;
        }
    }

    void KinematicViewerSession::Impl::PersistConfigOnExit() {
        cfg.playback.trajectory_files.clear();
        for (const auto& entry : playback_state.trajectory_files) {
            cfg.playback.trajectory_files.push_back(entry.path);
        }
        cfg.playback.selected_index   = playback_state.selected_trajectory_index;
        cfg.playback.last_browser_dir = playback_state.trajectory_browser_dir;
        cfg.teach.program_files.clear();
        for (const auto& entry : teach_state.program_files) {
            cfg.teach.program_files.push_back(entry.path);
        }
        cfg.teach.selected_index   = teach_state.selected_program_index;
        cfg.teach.last_browser_dir = teach_state.program_browser_dir;
    }

    bool KinematicViewerSession::Initialize(const LaunchConfig& launch, KinematicApp* app, std::string* error_message) {
        if (impl_ == nullptr || app == nullptr) {
            if (error_message != nullptr) {
                *error_message = "会话未创建或应用未初始化";
            }
            return false;
        }
        impl_->launch         = launch;
        impl_->cfg            = launch.config;
        impl_->urdf_path      = launch.urdfPath.empty() ? impl_->cfg.robot.urdf_path : launch.urdfPath;
        impl_->app            = app;
        impl_->window         = app->Window();
        impl_->ui_theme_index = KinematicUiThemeIndexFromName(impl_->cfg.ui.theme_preset);

        impl_->config_watcher = kinematic_viewer::ConfigWatcher(impl_->launch.configPath.empty() ? "" : impl_->launch.configPath);
        impl_->config_watcher.SetPollIntervalSec(2.0);
        impl_->config_watcher.SetOnChanged([impl = impl_](const KinematicViewerConfig& new_cfg) {
            impl->cfg            = new_cfg;
            impl->ui_theme_index = kinematic_viewer::KinematicUiThemeIndexFromName(impl->cfg.ui.theme_preset);
            kinematic_viewer::ApplyKinematicUiStyleByIndex(impl->ui_theme_index);
        });

        impl_->mesh_shader  = createKinematicMeshProgram();
        impl_->line_shader  = createKinematicLineProgram();
        impl_->point_shader = createKinematicPointProgram();
        impl_->line_renderer.init();

        impl_->render_loop.line_renderer = &impl_->line_renderer;

        if (!InitUserObstacleGpuMeshes(&impl_->obstacle_meshes)) {
            std::cerr << "InitUserObstacleGpuMeshes failed\n";
        }

        if (!impl_->scene.loadURDF(impl_->urdf_path)) {
            if (error_message != nullptr) {
                *error_message = std::string("Failed to load URDF: ") + impl_->urdf_path;
            }
            std::cerr << "Failed to load URDF: " << impl_->urdf_path << "\n";
            return false;
        }

        impl_->camera.distance     = impl_->cfg.camera.distance;
        impl_->camera.yaw          = impl_->cfg.camera.yaw;
        impl_->camera.pitch        = impl_->cfg.camera.pitch;
        impl_->camera.target       = impl_->cfg.camera.target;
        impl_->camera.rotate_speed = impl_->cfg.camera.rotate_speed;
        impl_->camera.zoom_scale   = impl_->cfg.camera.zoom_scale;
        impl_->camera.dolly_scale  = impl_->cfg.camera.dolly_scale;
        impl_->camera.pan_scale    = impl_->cfg.camera.pan_scale;
        impl_->camera.min_distance = impl_->cfg.camera.min_distance;
        impl_->camera.max_distance = impl_->cfg.camera.max_distance;

        impl_->ui_state.lock_base = impl_->cfg.ui.fix_base_like_mujoco;
        impl_->ui_state.mobile_base_drag_available =
            impl_->cfg.ui.enable_mobile_base_drag &&
            session_internal::IsMobileBaseDragRobot(impl_->urdf_path, impl_->cfg.ui.mobile_base_robots);
        impl_->ui_state.mobile_base_drag_enabled = impl_->ui_state.mobile_base_drag_available;
        auto appendJointInputGroup               = [&](const std::string& name, const std::vector<std::string>& joint_names) {
            if (joint_names.empty()) {
                return;
            }
            for (const auto& existing : impl_->ui_state.joint_input_groups) {
                if (existing.name == name) {
                    return;
                }
            }
            ViewerState::JointInputGroup group;
            group.name        = name;
            group.joint_names = joint_names;
            impl_->ui_state.joint_input_groups.push_back(std::move(group));
        };
        appendJointInputGroup("head", impl_->cfg.initial_pose.head_joint_names);
        appendJointInputGroup("leg", impl_->cfg.initial_pose.leg_joint_names);
        appendJointInputGroup("left_arm", impl_->cfg.initial_pose.left_arm_joint_names);
        appendJointInputGroup("right_arm", impl_->cfg.initial_pose.right_arm_joint_names);
        impl_->scene.setFixedBaseMode(impl_->ui_state.lock_base);

        impl_->point_cloud_state.visible       = impl_->cfg.point_cloud.visible;
        impl_->point_cloud_state.voxel_size    = impl_->cfg.point_cloud.voxel_size;
        impl_->point_cloud_state.max_points    = impl_->cfg.point_cloud.max_points;
        impl_->point_cloud_state.x_min         = impl_->cfg.point_cloud.x_min;
        impl_->point_cloud_state.x_max         = impl_->cfg.point_cloud.x_max;
        impl_->point_cloud_state.y_min         = impl_->cfg.point_cloud.y_min;
        impl_->point_cloud_state.y_max         = impl_->cfg.point_cloud.y_max;
        impl_->point_cloud_state.z_min         = impl_->cfg.point_cloud.z_min;
        impl_->point_cloud_state.z_max         = impl_->cfg.point_cloud.z_max;
        impl_->point_cloud_state.offset_x      = impl_->cfg.point_cloud.offset_x;
        impl_->point_cloud_state.offset_y      = impl_->cfg.point_cloud.offset_y;
        impl_->point_cloud_state.offset_yaw    = impl_->cfg.point_cloud.offset_yaw;
        impl_->point_cloud_state.point_size_px = impl_->cfg.point_cloud.point_size_px;
        if (impl_->cfg.point_cloud.color_mode == "intensity") {
            impl_->point_cloud_state.color_mode = 2;
        } else if (impl_->cfg.point_cloud.color_mode == "flat") {
            impl_->point_cloud_state.color_mode = 0;
        } else {
            impl_->point_cloud_state.color_mode = 1;
        }
        impl_->point_cloud_state.build_esdf           = impl_->cfg.point_cloud.build_esdf;
        impl_->point_cloud_state.esdf_resolution      = impl_->cfg.point_cloud.esdf_resolution;
        impl_->point_cloud_state.esdf_map_origin[0]   = impl_->cfg.point_cloud.esdf_map_origin[0];
        impl_->point_cloud_state.esdf_map_origin[1]   = impl_->cfg.point_cloud.esdf_map_origin[1];
        impl_->point_cloud_state.esdf_map_origin[2]   = impl_->cfg.point_cloud.esdf_map_origin[2];
        impl_->point_cloud_state.esdf_map_size[0]     = impl_->cfg.point_cloud.esdf_map_size[0];
        impl_->point_cloud_state.esdf_map_size[1]     = impl_->cfg.point_cloud.esdf_map_size[1];
        impl_->point_cloud_state.esdf_map_size[2]     = impl_->cfg.point_cloud.esdf_map_size[2];
        impl_->point_cloud_state.esdf_use_fixed_map   = impl_->cfg.point_cloud.esdf_use_fixed_map;
        impl_->point_cloud_state.esdf_max_visual_dist = impl_->cfg.point_cloud.esdf_max_visual_dist;
        impl_->point_cloud_state.esdf_visual_stride   = impl_->cfg.point_cloud.esdf_visual_stride;
        impl_->point_cloud_state.esdf_visual_mode     = kinematic_viewer::EsdfVisualModeFromString(impl_->cfg.point_cloud.esdf_visual_mode);
        impl_->point_cloud_state.esdf_color_mode      = kinematic_viewer::EsdfColorModeFromString(impl_->cfg.point_cloud.esdf_color_mode);
        impl_->point_cloud_state.esdf_z_slice_enable  = impl_->cfg.point_cloud.esdf_z_slice_enable;
        impl_->point_cloud_state.esdf_z_slice_m       = impl_->cfg.point_cloud.esdf_z_slice_m;
        impl_->point_cloud_state.esdf_use_raycast     = impl_->cfg.point_cloud.esdf_use_raycast;
        impl_->point_cloud_state.esdf_ray_origin[0]   = impl_->cfg.point_cloud.esdf_ray_origin[0];
        impl_->point_cloud_state.esdf_ray_origin[1]   = impl_->cfg.point_cloud.esdf_ray_origin[1];
        impl_->point_cloud_state.esdf_ray_origin[2]   = impl_->cfg.point_cloud.esdf_ray_origin[2];
        impl_->point_cloud_state.esdf_ray_origin_auto = impl_->cfg.point_cloud.esdf_ray_origin_auto;
        impl_->point_cloud_state.esdf_min_ray_length  = impl_->cfg.point_cloud.esdf_min_ray_length;
        impl_->point_cloud_state.esdf_max_ray_length  = impl_->cfg.point_cloud.esdf_max_ray_length;
        if (!impl_->cfg.point_cloud.file_path.empty()) {
            std::snprintf(impl_->point_cloud_state.file_path, sizeof(impl_->point_cloud_state.file_path), "%s",
                          impl_->cfg.point_cloud.file_path.c_str());
        }
        if (impl_->cfg.point_cloud.enable && impl_->cfg.point_cloud.auto_load_on_start && !impl_->cfg.point_cloud.file_path.empty()) {
            std::string pcd_path = impl_->cfg.point_cloud.file_path;
            if (!std::filesystem::path(pcd_path).is_absolute()) {
                std::filesystem::path resolved;
                if (!impl_->launch.configPath.empty()) {
                    resolved = std::filesystem::path(impl_->launch.configPath).parent_path().parent_path() / pcd_path;
                }
                if (!resolved.empty() && std::filesystem::exists(resolved)) {
                    pcd_path = resolved.lexically_normal().string();
                } else if (std::filesystem::exists(pcd_path)) {
                    pcd_path = std::filesystem::absolute(pcd_path).lexically_normal().string();
                }
            }
            PointCloudLoadOptions load_opt;
            load_opt.voxel_size           = impl_->point_cloud_state.voxel_size;
            load_opt.max_points           = impl_->point_cloud_state.max_points;
            load_opt.x_min                = impl_->point_cloud_state.x_min;
            load_opt.x_max                = impl_->point_cloud_state.x_max;
            load_opt.y_min                = impl_->point_cloud_state.y_min;
            load_opt.y_max                = impl_->point_cloud_state.y_max;
            load_opt.z_min                = impl_->point_cloud_state.z_min;
            load_opt.z_max                = impl_->point_cloud_state.z_max;
            load_opt.color_mode           = PointCloudColorModeFromString(impl_->cfg.point_cloud.color_mode);
            load_opt.build_esdf           = impl_->cfg.point_cloud.build_esdf;
            load_opt.esdf_resolution      = impl_->cfg.point_cloud.esdf_resolution;
            load_opt.esdf_map_origin[0]   = impl_->cfg.point_cloud.esdf_map_origin[0];
            load_opt.esdf_map_origin[1]   = impl_->cfg.point_cloud.esdf_map_origin[1];
            load_opt.esdf_map_origin[2]   = impl_->cfg.point_cloud.esdf_map_origin[2];
            load_opt.esdf_map_size[0]     = impl_->cfg.point_cloud.esdf_map_size[0];
            load_opt.esdf_map_size[1]     = impl_->cfg.point_cloud.esdf_map_size[1];
            load_opt.esdf_map_size[2]     = impl_->cfg.point_cloud.esdf_map_size[2];
            load_opt.esdf_use_fixed_map   = impl_->cfg.point_cloud.esdf_use_fixed_map;
            load_opt.esdf_max_visual_dist = impl_->cfg.point_cloud.esdf_max_visual_dist;
            load_opt.esdf_visual_stride   = impl_->cfg.point_cloud.esdf_visual_stride;
            load_opt.esdf_visual_mode     = kinematic_viewer::EsdfVisualModeFromString(impl_->cfg.point_cloud.esdf_visual_mode);
            load_opt.esdf_color_mode      = kinematic_viewer::EsdfColorModeFromString(impl_->cfg.point_cloud.esdf_color_mode);
            load_opt.esdf_z_slice_enable  = impl_->cfg.point_cloud.esdf_z_slice_enable;
            load_opt.esdf_z_slice_m       = impl_->cfg.point_cloud.esdf_z_slice_m;
            load_opt.esdf_use_raycast     = impl_->cfg.point_cloud.esdf_use_raycast;
            load_opt.esdf_ray_origin[0]   = impl_->cfg.point_cloud.esdf_ray_origin[0];
            load_opt.esdf_ray_origin[1]   = impl_->cfg.point_cloud.esdf_ray_origin[1];
            load_opt.esdf_ray_origin[2]   = impl_->cfg.point_cloud.esdf_ray_origin[2];
            load_opt.esdf_ray_origin_auto = impl_->cfg.point_cloud.esdf_ray_origin_auto;
            load_opt.esdf_min_ray_length  = impl_->cfg.point_cloud.esdf_min_ray_length;
            load_opt.esdf_max_ray_length  = impl_->cfg.point_cloud.esdf_max_ray_length;
            if (impl_->cfg.point_cloud.build_esdf && impl_->cfg.point_cloud.point_size_px >= 2.0f) {
                impl_->point_cloud_state.point_size_px = impl_->cfg.point_cloud.point_size_px;
            }
            std::string status;
            if (impl_->point_cloud_layer.LoadFromFile(pcd_path, load_opt, &status)) {
                impl_->point_cloud_state.loaded          = true;
                impl_->point_cloud_state.rendered_points = impl_->point_cloud_layer.gpuPointCount();
                std::snprintf(impl_->point_cloud_state.file_path, sizeof(impl_->point_cloud_state.file_path), "%s", pcd_path.c_str());
            }
            std::snprintf(impl_->point_cloud_state.last_status, sizeof(impl_->point_cloud_state.last_status), "%s", status.c_str());
        }

        // ---------------------------------------------------------------------------
        // Panel plugin registry — loads panel .so files from lib/ by id or path.
        // ---------------------------------------------------------------------------
        {
            // Determine lib search directory: <exe>/../lib
            std::string exe_dir;
            {
                char buf[4096] = {};
                ssize_t len    = ::readlink("/proc/self/exe", buf, sizeof(buf) - 1);
                if (len > 0) {
                    buf[len] = '\0';
                    exe_dir  = std::filesystem::path(buf).parent_path().string();
                }
            }
            const std::vector<std::string> panel_search_dirs = {
                exe_dir + "/../lib",
                exe_dir + "/lib",
                exe_dir,
            };

            std::vector<std::string> specs = impl_->cfg.ui.sidebar_panels;
            if (specs.empty()) {
                specs = {"scene", "ik", "playback", "safety", "joint", "tf", "obstacle", "planner", "teach", "point_cloud"};
            }
            impl_->panel_registry.LoadFromConfig(specs, panel_search_dirs);
            if (impl_->panel_registry.Count() < static_cast<int>(specs.size())) {
                std::cerr << "[robot_kinematic_viewer] sidebar panels: loaded " << impl_->panel_registry.Count() << "/" << specs.size()
                          << ". Missing plugins are usually stale librkv_panel_*.so — run a full build.\n";
            }
        }
        {
            const int preferred          = impl_->panel_registry.IndexOf("joint");
            impl_->ui_state.sidebar_page = preferred >= 0 ? preferred : 0;
        }

        // Restore trajectory file list from config
        for (const auto& path : impl_->cfg.playback.trajectory_files) {
            kinematic_viewer::TrajectoryFileEntry entry;
            entry.path   = path;
            entry.status = "未加载";
            entry.loaded = false;
            impl_->playback_state.trajectory_files.push_back(std::move(entry));
        }
        if (!impl_->playback_state.trajectory_files.empty()) {
            impl_->playback_state.selected_trajectory_index =
                std::clamp(impl_->cfg.playback.selected_index, 0, static_cast<int>(impl_->playback_state.trajectory_files.size()) - 1);
            std::snprintf(impl_->playback_state.trajectory_file_path, sizeof(impl_->playback_state.trajectory_file_path), "%s",
                          impl_->playback_state.trajectory_files[impl_->playback_state.selected_trajectory_index].path.c_str());
        }
        {
            auto isExistingDir = [](const std::string& path) -> bool {
                std::error_code ec;
                return std::filesystem::exists(path, ec) && std::filesystem::is_directory(path, ec);
            };
            std::string initial_browser_dir;
            if (!impl_->cfg.playback.last_browser_dir.empty() && isExistingDir(impl_->cfg.playback.last_browser_dir)) {
                initial_browser_dir = kinematic_viewer::NormalizePath(impl_->cfg.playback.last_browser_dir);
            } else {
                const std::string trajectories_dir = kinematic_viewer::NormalizePath("config/trajectories");
                if (isExistingDir(trajectories_dir)) {
                    initial_browser_dir = trajectories_dir;
                } else if (!impl_->playback_state.trajectory_files.empty()) {
                    initial_browser_dir = kinematic_viewer::NormalizePath(
                        std::filesystem::path(impl_->playback_state.trajectory_files.front().path).parent_path().string());
                } else {
                    const char* home    = std::getenv("HOME");
                    initial_browser_dir = (home != nullptr && home[0] != '\0')
                                              ? kinematic_viewer::NormalizePath(home)
                                              : kinematic_viewer::NormalizePath(std::filesystem::current_path().string());
                }
            }
            std::snprintf(impl_->playback_state.trajectory_browser_dir, sizeof(impl_->playback_state.trajectory_browser_dir), "%s",
                          initial_browser_dir.c_str());
        }
        for (const auto& path : impl_->cfg.teach.program_files) {
            kinematic_viewer::TeachFileEntry entry;
            entry.path   = path;
            entry.status = "未加载";
            entry.loaded = false;
            impl_->teach_state.program_files.push_back(std::move(entry));
        }
        if (!impl_->teach_state.program_files.empty()) {
            impl_->teach_state.selected_program_index =
                std::clamp(impl_->cfg.teach.selected_index, 0, static_cast<int>(impl_->teach_state.program_files.size()) - 1);
            std::snprintf(impl_->teach_state.program_file_path, sizeof(impl_->teach_state.program_file_path), "%s",
                          impl_->teach_state.program_files[impl_->teach_state.selected_program_index].path.c_str());
        }
        {
            auto isExistingDir = [](const std::string& path) -> bool {
                std::error_code ec;
                return std::filesystem::exists(path, ec) && std::filesystem::is_directory(path, ec);
            };
            std::string teach_browser_dir;
            if (!impl_->cfg.teach.last_browser_dir.empty() && isExistingDir(impl_->cfg.teach.last_browser_dir)) {
                teach_browser_dir = kinematic_viewer::NormalizePath(impl_->cfg.teach.last_browser_dir);
            } else {
                const std::string teach_dir = kinematic_viewer::NormalizePath("config/teach");
                teach_browser_dir           = isExistingDir(teach_dir) ? teach_dir : impl_->playback_state.trajectory_browser_dir;
            }
            std::snprintf(impl_->teach_state.program_browser_dir, sizeof(impl_->teach_state.program_browser_dir), "%s",
                          teach_browser_dir.c_str());
        }
        if (impl_->teach_state.selected_program_index >= 0) {
            std::string teach_load_error;
            auto& selected_entry = impl_->teach_state.program_files[static_cast<size_t>(impl_->teach_state.selected_program_index)];
            if (LoadTeachProgramFromYaml(impl_->teach_state.program_file_path, &impl_->teach_state, &teach_load_error)) {
                selected_entry.status        = "加载成功";
                selected_entry.loaded        = true;
                impl_->teach_state.io_status = "启动已加载: " + impl_->teach_state.program_name;
            } else {
                selected_entry.status        = "加载失败: " + teach_load_error;
                selected_entry.loaded        = false;
                impl_->teach_state.io_status = selected_entry.status;
            }
        }
        {
            impl_->ik_state.solve_mode = impl_->cfg.ik.mode;
            std::transform(impl_->ik_state.solve_mode.begin(), impl_->ik_state.solve_mode.end(), impl_->ik_state.solve_mode.begin(),
                           [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
            if (impl_->ik_state.solve_mode != "single_chain" && impl_->ik_state.solve_mode != "full_body") {
                impl_->ik_state.solve_mode = "single_chain";
            }
            impl_->ik_state.full_body_backend = impl_->cfg.ik.full_body_backend;
            std::transform(impl_->ik_state.full_body_backend.begin(), impl_->ik_state.full_body_backend.end(),
                           impl_->ik_state.full_body_backend.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
            if (impl_->ik_state.full_body_backend != "flex_ik" && impl_->ik_state.full_body_backend != "wbc_chain_ik") {
                impl_->ik_state.full_body_backend = "flex_ik";
            }
            impl_->ik_state.full_body_iterations = impl_->cfg.ik.full_body_iterations;
            impl_->ik_state.solver.setFullBodyBackend(impl_->ik_state.full_body_backend);
            impl_->ik_state.solver.initialize(impl_->urdf_path, impl_->cfg.ik.chains);
            impl_->ik_state.chains.clear();
            for (int i = 0; i < impl_->ik_state.solver.chainCount(); ++i) {
                impl_->ik_state.chains.push_back(impl_->ik_state.solver.chainStatus(i));
            }
            if (!impl_->ik_state.chains.empty()) {
                impl_->ik_state.selected_chain =
                    std::clamp(impl_->ik_state.selected_chain, 0, static_cast<int>(impl_->ik_state.chains.size()) - 1);
            }
            impl_->ik_state.marker_targets.resize(impl_->ik_state.chains.size());
        }
        impl_->ik_state.use_external_target = false;

        if (!impl_->ik_state.chains.empty()) {
            impl_->ik_controller.LoadActiveMarkerFromTarget(&impl_->scene);
        }
        return true;
    }

    void KinematicViewerSession::Run(KinematicApp& app) {
        impl_->last_frame_sec = glfwGetTime();
        while (!app.ShouldClose()) {
            app.PollEvents();
            impl_->config_watcher.Poll();
            const double now_sec  = glfwGetTime();
            const double dt_sec   = std::max(0.0, now_sec - impl_->last_frame_sec);
            impl_->last_frame_sec = now_sec;
            double mouse_x        = 0.0;
            double mouse_y        = 0.0;
            glfwGetCursorPos(impl_->window, &mouse_x, &mouse_y);
            TickFrame(app, dt_sec, now_sec, mouse_x, mouse_y);
        }
        impl_->PersistConfigOnExit();
    }

    void KinematicViewerSession::TickFrame(KinematicApp& app, double dt_sec, double now_sec, double mouse_x, double mouse_y) {
        impl_->ApplyDeferredInitialPose(now_sec);

        KinematicViewerSession::FrameContext frame = impl_->BeginUiFrame(mouse_x, mouse_y);
        frame.dt_sec                               = dt_sec;
        frame.now_sec                              = now_sec;

        impl_->HandleFrameHotkeys(&frame);

        KinematicInputHandler::UpdateContext input_ctx;
        impl_->UpdateViewportInput(&frame, &input_ctx);
        impl_->AdvanceSimulation(&frame);
        impl_->RenderViewport3D(frame);
        impl_->HandleImGuizmos(&frame);
        impl_->HandleViewportPickingAndCamera(&frame, &input_ctx);
        impl_->RenderViewportHud(frame);
        impl_->RenderSidebar(&frame);
        impl_->PresentFrame(app, frame);
    }

}  // namespace kinematic_viewer
