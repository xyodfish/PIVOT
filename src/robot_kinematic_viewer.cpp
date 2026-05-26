#include "kinematic_viewer/kinematic_app.h"
#include "kinematic_viewer/kinematic_bootstrap.h"
#include "kinematic_viewer/kinematic_collision_monitor.h"
#include "kinematic_viewer/kinematic_ik_controller.h"
#include "kinematic_viewer/kinematic_initial_pose.h"
#include "kinematic_viewer/kinematic_line_renderer.h"
#include "kinematic_viewer/kinematic_marker_target_state.h"
#include "kinematic_viewer/kinematic_marker_utils.h"
#include "kinematic_viewer/kinematic_playback.h"
#include "kinematic_viewer/kinematic_playback_state_machine.h"
#include "kinematic_viewer/kinematic_render_loop.h"
#include "kinematic_viewer/kinematic_input_handler.h"
#include "kinematic_viewer/kinematic_link_kinematics.h"
#include "kinematic_viewer/kinematic_link_inspector.h"
#include "kinematic_viewer/kinematic_config_watcher.h"
#include "kinematic_viewer/kinematic_ros_bridge.h"
#include "kinematic_viewer/kinematic_runtime_state.h"
#include "kinematic_viewer/kinematic_shader_utils.h"
#include "kinematic_viewer/kinematic_robot_tree_panel.h"
#include "kinematic_viewer/kinematic_sidebar_layout.h"
#include "kinematic_viewer/kinematic_sidebar_panels.h"
#include "kinematic_viewer/kinematic_string_utils.h"
#include "kinematic_viewer/kinematic_ui_feedback.h"
#include "kinematic_viewer/kinematic_ui_theme.h"
#include "kinematic_viewer/kinematic_user_obstacles.h"
#include "kinematic_viewer/kinematic_viewer_config.h"
#include "teleop_viewer/ik_solver.h"
#include "teleop_viewer/scene.h"

#include "ImGuizmo.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <glad/glad.h>

#include <GLFW/glfw3.h>

#include <geometry_msgs/PoseStamped.h>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

using kinematic_viewer::appendCircle;
using kinematic_viewer::appendMarkerAxes;
using kinematic_viewer::CollisionMonitor;
using kinematic_viewer::CollisionMonitorResult;
using kinematic_viewer::CollisionMonitorState;
using kinematic_viewer::createKinematicLineProgram;
using kinematic_viewer::createKinematicMeshProgram;
using kinematic_viewer::DebugPlaybackState;
using kinematic_viewer::DestroyUserObstacleGpuMeshes;
using kinematic_viewer::distancePointToSegment2D;
using kinematic_viewer::DrawUserObstacles;
using kinematic_viewer::GetScrollDelta;
using kinematic_viewer::IkState;
using kinematic_viewer::InitialPoseApplyResult;
using kinematic_viewer::InitUserObstacleGpuMeshes;
using kinematic_viewer::KinematicApp;
using kinematic_viewer::KinematicIkController;
using kinematic_viewer::KinematicInputHandler;
using kinematic_viewer::KinematicLineRenderer;
using kinematic_viewer::KinematicLineVertex;
using kinematic_viewer::KinematicRenderLoop;
using kinematic_viewer::KinematicRosBridge;
using kinematic_viewer::KinematicUiFeedback;
using kinematic_viewer::KinematicViewerConfig;
using kinematic_viewer::LaunchConfig;
using kinematic_viewer::LoadLaunchConfigFromArgs;
using kinematic_viewer::markerWorldMatrix;
using kinematic_viewer::MergeUserObstaclesIntoCollisionResult;
using kinematic_viewer::LinkKinematicsAnalyzer;
using kinematic_viewer::RenderJointPanel;
using kinematic_viewer::FocusCameraOnLink;
using kinematic_viewer::RenderLinkInspectorPanel;
using kinematic_viewer::RenderObstaclePanel;
using kinematic_viewer::RenderPlaybackPanel;
using kinematic_viewer::RenderSafetyPanel;
using kinematic_viewer::RenderRobotTreePanel;
using kinematic_viewer::RenderScenePanel;
using kinematic_viewer::RenderTfPanel;
using kinematic_viewer::TrajectoryPlayer;
using kinematic_viewer::UiSemanticLevel;
using kinematic_viewer::UserObstacleGpuMeshes;
using kinematic_viewer::ViewerState;
using kinematic_viewer::worldToScreen;
using kinematic_viewer::wrapDeltaDeg;
using teleop_viewer::IkSolveStats;
using teleop_viewer::OrbitCamera;
using teleop_viewer::RobotScene;

namespace robot_kinematic_viewer_internal {

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

    float ObstaclePickRadius(const kinematic_viewer::UserObstacleItem& obs) {
        if (obs.kind == kinematic_viewer::UserObstacleItem::Kind::Sphere) {
            return std::max(1e-4f, obs.params.x);
        }
        if (obs.kind == kinematic_viewer::UserObstacleItem::Kind::Box) {
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

}  // namespace robot_kinematic_viewer_internal

using robot_kinematic_viewer_internal::ComputeWorldRayFromScreen;
using robot_kinematic_viewer_internal::IntersectRaySphere;
using robot_kinematic_viewer_internal::ObstaclePickRadius;

int main(int argc, char** argv) {
    LaunchConfig launch;
    std::string launchError;
    if (!LoadLaunchConfigFromArgs(argc, argv, &launch, &launchError)) {
        if (!launchError.empty()) {
            std::cerr << launchError << std::endl;
        }
        return 1;
    }
    KinematicViewerConfig cfg = launch.config;
    std::string urdf_path     = launch.urdfPath.empty() ? cfg.robot.urdf_path : launch.urdfPath;

    KinematicApp app;
    {
        auto init_result = app.Initialize(cfg);
        if (!init_result.success) {
            std::cerr << init_result.error << "\n";
            return 1;
        }
    }
    int ui_theme_index = kinematic_viewer::KinematicUiThemeIndexFromName(cfg.ui.theme_preset);
    GLFWwindow* window = app.Window();

    kinematic_viewer::ConfigWatcher config_watcher(launch.configPath.empty() ? "" : launch.configPath);
    config_watcher.SetPollIntervalSec(2.0);
    config_watcher.SetOnChanged([&](const KinematicViewerConfig& new_cfg) {
        cfg            = new_cfg;
        ui_theme_index = kinematic_viewer::KinematicUiThemeIndexFromName(cfg.ui.theme_preset);
        kinematic_viewer::ApplyKinematicUiStyleByIndex(ui_theme_index);
    });

    GLuint mesh_shader = createKinematicMeshProgram();
    GLuint line_shader = createKinematicLineProgram();
    KinematicLineRenderer line_renderer;
    line_renderer.init();

    KinematicRenderLoop render_loop;
    render_loop.line_renderer = &line_renderer;

    UserObstacleGpuMeshes obstacle_meshes;
    if (!InitUserObstacleGpuMeshes(&obstacle_meshes)) {
        std::cerr << "InitUserObstacleGpuMeshes failed\n";
    }

    RobotScene scene;
    if (!scene.loadURDF(urdf_path)) {
        std::cerr << "Failed to load URDF: " << urdf_path << "\n";
        return 1;
    }

    OrbitCamera camera;
    camera.distance     = cfg.camera.distance;
    camera.yaw          = cfg.camera.yaw;
    camera.pitch        = cfg.camera.pitch;
    camera.target       = cfg.camera.target;
    camera.rotate_speed = cfg.camera.rotate_speed;
    camera.zoom_scale   = cfg.camera.zoom_scale;
    camera.dolly_scale  = cfg.camera.dolly_scale;
    camera.pan_scale    = cfg.camera.pan_scale;
    camera.min_distance = cfg.camera.min_distance;
    camera.max_distance = cfg.camera.max_distance;

    ViewerState ui_state;
    kinematic_viewer::PathPlannerUiState path_planner_ui;
    ui_state.lock_base         = cfg.ui.fix_base_like_mujoco;
    auto appendJointInputGroup = [&](const std::string& name, const std::vector<std::string>& joint_names) {
        if (joint_names.empty()) {
            return;
        }
        for (const auto& existing : ui_state.joint_input_groups) {
            if (existing.name == name) {
                return;
            }
        }
        ViewerState::JointInputGroup group;
        group.name        = name;
        group.joint_names = joint_names;
        ui_state.joint_input_groups.push_back(std::move(group));
    };
    appendJointInputGroup("head", cfg.initial_pose.head_joint_names);
    appendJointInputGroup("leg", cfg.initial_pose.leg_joint_names);
    appendJointInputGroup("left_arm", cfg.initial_pose.left_arm_joint_names);
    appendJointInputGroup("right_arm", cfg.initial_pose.right_arm_joint_names);
    scene.setFixedBaseMode(ui_state.lock_base);

    KinematicRosBridge ros_bridge(cfg.ros.enable);
    ros_bridge.initialize(argc, argv);

    std::string ikModeParam;
    std::string fullBodyBackendParam;
    int fullBodyIterationsParam = cfg.ik.full_body_iterations;
    if (ros_bridge.getParam("ik_mode", &ikModeParam)) {
        cfg.ik.mode = ikModeParam;
    }
    if (ros_bridge.getParam("ik_full_body_backend", &fullBodyBackendParam)) {
        cfg.ik.full_body_backend = fullBodyBackendParam;
    }
    if (ros_bridge.getParam("ik_full_body_iterations", &fullBodyIterationsParam)) {
        cfg.ik.full_body_iterations = std::max(1, fullBodyIterationsParam);
    }

    IkState ik_state;
    DebugPlaybackState playback_state;
    // Restore trajectory file list from config
    for (const auto& path : cfg.playback.trajectory_files) {
        kinematic_viewer::TrajectoryFileEntry entry;
        entry.path   = path;
        entry.status = "未加载";
        entry.loaded = false;
        playback_state.trajectory_files.push_back(std::move(entry));
    }
    if (!playback_state.trajectory_files.empty()) {
        playback_state.selected_trajectory_index =
            std::clamp(cfg.playback.selected_index, 0, static_cast<int>(playback_state.trajectory_files.size()) - 1);
        std::snprintf(playback_state.trajectory_file_path, sizeof(playback_state.trajectory_file_path), "%s",
                      playback_state.trajectory_files[playback_state.selected_trajectory_index].path.c_str());
    }
    kinematic_viewer::PlaybackStateMachine playback_sm(&playback_state);
    CollisionMonitorState collision_state;
    TrajectoryPlayer trajectory_player;
    CollisionMonitor collision_monitor;
    LinkKinematicsAnalyzer link_kinematics_analyzer;
    KinematicUiFeedback ui_feedback;
    kinematic_viewer::KinematicIkController ik_controller(&ik_state);
    std::string last_playback_io_status;
    bool initial_pose_auto_apply_pending = cfg.initial_pose.enable && cfg.initial_pose.auto_apply_on_start;
    {
        ik_state.solve_mode = cfg.ik.mode;
        std::transform(ik_state.solve_mode.begin(), ik_state.solve_mode.end(), ik_state.solve_mode.begin(),
                       [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
        if (ik_state.solve_mode != "single_chain" && ik_state.solve_mode != "full_body") {
            ik_state.solve_mode = "single_chain";
        }
        ik_state.full_body_backend = cfg.ik.full_body_backend;
        std::transform(ik_state.full_body_backend.begin(), ik_state.full_body_backend.end(), ik_state.full_body_backend.begin(),
                       [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
        if (ik_state.full_body_backend != "flex_ik" && ik_state.full_body_backend != "wbc_chain_ik") {
            ik_state.full_body_backend = "flex_ik";
        }
        ik_state.full_body_iterations = cfg.ik.full_body_iterations;
        ik_state.solver.setFullBodyBackend(ik_state.full_body_backend);
        ik_state.solver.initialize(urdf_path, cfg.ik.chains);
        ik_state.chains.clear();
        for (int i = 0; i < ik_state.solver.chainCount(); ++i) {
            ik_state.chains.push_back(ik_state.solver.chainStatus(i));
        }
        ros_bridge.param<int>("ik_selected_chain", &ik_state.selected_chain, ik_state.selected_chain);
        if (!ik_state.chains.empty()) {
            ik_state.selected_chain = std::clamp(ik_state.selected_chain, 0, static_cast<int>(ik_state.chains.size()) - 1);
        }
        ik_state.marker_targets.resize(ik_state.chains.size());
    }

    ros_bridge.param<std::string>("ik_target_pose_topic", &ik_state.external_target_topic, ik_state.external_target_topic);
    ros_bridge.param<std::string>("ik_target_pose_frame", &ik_state.external_target_expected_frame,
                                  ik_state.external_target_expected_frame);
    ros_bridge.param<bool>("enable_external_ik_target", &ik_state.use_external_target, ik_state.use_external_target);
    ros_bridge.param<bool>("external_ik_target_position_only", &ik_state.external_target_position_only,
                           ik_state.external_target_position_only);
    if (!ros_bridge.enabled()) {
        ik_state.use_external_target = false;
    }

    ros_bridge.subscribeExternalTarget(ik_state.external_target_topic, 20, [&](const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (msg == nullptr) {
            return;
        }
        const std::string frame = msg->header.frame_id;
        if (!ik_state.external_target_expected_frame.empty() && frame != ik_state.external_target_expected_frame) {
            return;
        }
        ik_state.external_target_pos  = glm::vec3(static_cast<float>(msg->pose.position.x), static_cast<float>(msg->pose.position.y),
                                                  static_cast<float>(msg->pose.position.z));
        ik_state.external_target_quat = glm::quat(static_cast<float>(msg->pose.orientation.w), static_cast<float>(msg->pose.orientation.x),
                                                  static_cast<float>(msg->pose.orientation.y), static_cast<float>(msg->pose.orientation.z));
        if (glm::length(ik_state.external_target_quat) > 1e-6f) {
            ik_state.external_target_quat = glm::normalize(ik_state.external_target_quat);
        } else {
            ik_state.external_target_quat = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
        }
        ik_state.external_target_last_frame    = frame;
        ik_state.external_target_last_recv_sec = ros_bridge.nowSec();
        ik_state.external_target_received      = true;
        ik_state.external_target_dirty         = true;
    });

    double prev_x                 = 0.0;
    double prev_y                 = 0.0;
    bool first_mouse              = true;
    bool obstacle_pick_left_prev  = false;
    bool obstacle_gizmo_was_using = false;
    bool obstacle_gizmo_was_over  = false;
    kinematic_viewer::KinematicInputHandler input_handler;
    double last_frame_sec = glfwGetTime();

    if (!ik_state.chains.empty()) {
        ik_controller.LoadActiveMarkerFromTarget(&scene);
    }

    auto applyConfiguredInitialPose = [&]() -> InitialPoseApplyResult {
        InitialPoseApplyResult result = kinematic_viewer::ApplyConfiguredInitialPose(cfg.initial_pose, &scene);
        ik_state.marker_initialized   = false;
        if (!ik_state.chains.empty()) {
            ik_controller.LoadActiveMarkerFromTarget(&scene);
        }
        return result;
    };

    while (!app.ShouldClose()) {
        ros_bridge.spinOnce();
        app.PollEvents();
        config_watcher.Poll();
        double now_sec = glfwGetTime();
        double dt_sec  = std::max(0.0, now_sec - last_frame_sec);
        last_frame_sec = now_sec;

        if (initial_pose_auto_apply_pending) {
            InitialPoseApplyResult result = applyConfiguredInitialPose();
            UiSemanticLevel level         = result.missing_joint_count > 0 ? UiSemanticLevel::Warning : UiSemanticLevel::Success;
            ui_feedback.Push(level, std::string("初始位姿加载: ") + result.detail, now_sec, 4.0);
            initial_pose_auto_apply_pending = false;
        }

        double x = 0.0, y = 0.0;
        glfwGetCursorPos(window, &x, &y);

        int fb_w = 0, fb_h = 0;
        glfwGetFramebufferSize(window, &fb_w, &fb_h);
        float panel_min      = 280.0f;
        float panel_max      = std::max(panel_min, static_cast<float>(fb_w) - 320.0f);
        ui_state.panel_width = std::clamp(ui_state.panel_width, panel_min, panel_max);
        int panel_w          = static_cast<int>(ui_state.panel_width);
        int viewport_w       = std::max(1, fb_w - panel_w);
        int viewport_h       = std::max(1, fb_h);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        ImGuizmo::BeginFrame();

        const bool sidebar_hotkeys_enabled = !ImGui::GetIO().WantTextInput && !ImGui::GetIO().WantCaptureKeyboard;
        ui_state.sidebar_page              = input_handler.HandleSidebarHotkeys(ui_state.sidebar_page, sidebar_hotkeys_enabled);

        KinematicInputHandler::UpdateContext input_ctx;
        input_ctx.mouse_x             = x;
        input_ctx.mouse_y             = y;
        input_ctx.viewport_w          = viewport_w;
        input_ctx.viewport_h          = viewport_h;
        input_ctx.imgui_wants_mouse   = ImGui::GetIO().WantCaptureMouse;
        input_ctx.panel_resize_active = ui_state.panel_resize_active;
        input_ctx.ik_gizmo_using      = ik_state.gizmo_was_using;
        input_ctx.ik_gizmo_over       = ik_state.gizmo_was_over;
        input_ctx.obs_gizmo_using     = obstacle_gizmo_was_using;
        input_ctx.obs_gizmo_over      = obstacle_gizmo_was_over;
        input_ctx.ik_dragging_marker  = ik_state.dragging_marker;
        input_ctx.sidebar_page        = ui_state.sidebar_page;
        input_ctx.scroll_delta        = GetScrollDelta();
        input_ctx.left_mouse_down     = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
        input_ctx.middle_mouse_down   = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
        input_ctx.right_mouse_down    = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
        input_ctx.shift_key_down =
            glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;

        const bool was_playback_playing = playback_sm.IsPlaying();
        playback_sm.AdvanceTime(static_cast<float>(dt_sec));
        if (playback_sm.IsPlaying()) {
            trajectory_player.SampleAtCurrentTime(playback_state, &scene);
        }
        scene.setFixedBaseMode(ui_state.lock_base);
        scene.updateTransforms();
        CollisionMonitorResult collision_result = collision_monitor.Evaluate(collision_state, scene);
        if (collision_state.enable) {
            MergeUserObstaclesIntoCollisionResult(ui_state.user_obstacles, scene, collision_state.warning_distance_m,
                                                  collision_state.danger_distance_m, &collision_result);
        }
        collision_monitor.UpdateStateFromResult(collision_result, &collision_state);

        glm::mat4 proj =
            glm::perspective(glm::radians(50.0f), static_cast<float>(viewport_w) / static_cast<float>(viewport_h), 0.05f, 80.0f);
        glm::mat4 view = camera.viewMatrix();

        KinematicRenderLoop::Context render_ctx;
        render_ctx.viewport_w        = viewport_w;
        render_ctx.viewport_h        = viewport_h;
        render_ctx.mesh_shader       = mesh_shader;
        render_ctx.line_shader       = line_shader;
        render_ctx.scene             = &scene;
        render_ctx.ui_state          = &ui_state;
        render_ctx.ik_state          = &ik_state;
        render_ctx.collision_state   = &collision_state;
        render_ctx.collision_result  = &collision_result;
        render_ctx.obstacle_meshes   = &obstacle_meshes;
        render_ctx.camera            = &camera;
        render_ctx.planned_path      = &path_planner_ui.preview_waypoints;
        render_ctx.show_planned_path = path_planner_ui.show_preview;
        render_loop.Render(render_ctx);

        auto applyIkForActiveChain = [&](bool force_orientation_lock, bool fast_mode, bool prefer_position_only_target) -> bool {
            return ik_controller.ApplyIkForActiveChain(&scene, force_orientation_lock, fast_mode, prefer_position_only_target);
        };

        auto refineActiveChainToMarker = [&]() -> bool {
            return ik_controller.RefineActiveChainToMarker(&scene);
        };

        auto activeChainPositionErrorMmToMarker = [&]() -> float {
            return ik_controller.ActiveChainPositionErrorMmToMarker(&scene);
        };

        // External RViz interactive marker target -> local marker -> IK.
        ik_controller.ApplyExternalTarget(&scene);

        // RViz-like manipulator via ImGuizmo
        // Draw gizmo directly on the foreground drawlist of the 3D viewport area.
        ImGuizmo::SetOrthographic(false);
        ImGuizmo::AllowAxisFlip(false);
        ImGuizmo::SetDrawlist(ImGui::GetForegroundDrawList());
        ImGuizmo::SetRect(0.0f, 0.0f, static_cast<float>(viewport_w), static_cast<float>(viewport_h));

        const bool obstacle_page_active = (ui_state.sidebar_page == 0 || ui_state.sidebar_page == 6);
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
            bool obs_manipulated = ImGuizmo::Manipulate(glm::value_ptr(view), glm::value_ptr(proj), obs_op, obs_mode,
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

        ImGuizmo::SetGizmoSizeClipSpace(ik_state.gizmo_size_clip_space);
        if (!obstacle_edit_active && ik_state.selected_chain >= 0 && ik_state.selected_chain < static_cast<int>(ik_state.chains.size())) {
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
            bool manipulated = ImGuizmo::Manipulate(glm::value_ptr(view), glm::value_ptr(proj), op, mode, glm::value_ptr(gizmo_world),
                                                    glm::value_ptr(gizmo_delta), snap_ptr);
            bool gizmo_using = ImGuizmo::IsUsing();
            bool gizmo_over  = ImGuizmo::IsOver();
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
                if (ik_state.last_realtime_ik_apply_sec < 0.0 || (now_sec - ik_state.last_realtime_ik_apply_sec) >= interval_sec) {
                    applyIkForActiveChain(false, true, current_drag_position_only);
                    ik_state.last_realtime_ik_apply_sec = now_sec;
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

        // Click in 3D viewport to select nearest obstacle.
        glm::mat4 pick_proj =
            glm::perspective(glm::radians(50.0f), static_cast<float>(viewport_w) / static_cast<float>(viewport_h), 0.05f, 80.0f);
        glm::mat4 pick_view = camera.viewMatrix();
        auto obstacle_pick  = input_handler.UpdateObstaclePick(input_ctx, pick_view, pick_proj, ui_state.user_obstacles);
        if (obstacle_pick.picked) {
            ui_state.user_obstacles.selected_index = obstacle_pick.selected_index;
        }

        auto link_hover = input_handler.UpdateLinkHover(input_ctx, pick_view, pick_proj, &scene, glfwGetTime());
        if (!link_hover.throttle_skip) {
            ui_state.hovered_link = link_hover.picked ? link_hover.link_name : std::string();
        }

        auto link_pick = input_handler.UpdateLinkPick(input_ctx, pick_view, pick_proj, &scene);
        if (link_pick.picked) {
            ui_state.selected_link            = link_pick.link_name;
            ui_state.trajectory_min_surface_m = -1.0f;
            ui_state.selected_joint           = -1;
        }

        input_ctx.ik_gizmo_using     = ik_state.gizmo_was_using;
        input_ctx.ik_dragging_marker = ik_state.dragging_marker;
        input_ctx.obs_gizmo_using    = obstacle_gizmo_was_using;
        input_ctx.imgui_wants_mouse  = ImGui::GetIO().WantCaptureMouse;
        input_handler.UpdateCamera(&camera, input_ctx);

        // Marker hover/pick in viewport (screen-space) — disabled (false branch)
        if (false && ik_state.selected_chain >= 0 && ik_state.selected_chain < static_cast<int>(ik_state.chains.size()) &&
            !ImGui::GetIO().WantCaptureMouse) {
            // This block is disabled. If re-enabled, view/proj matrices need to be recomputed here
            // since they are no longer kept in the main loop scope after KinematicRenderLoop refactoring.
        }
        glViewport(viewport_w, 0, panel_w, fb_h);
        glDisable(GL_DEPTH_TEST);
        ImGui::SetNextWindowPos(ImVec2(static_cast<float>(viewport_w), 0.0f), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(static_cast<float>(panel_w), static_cast<float>(fb_h)), ImGuiCond_Always);
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
                ui_state.panel_width = std::clamp(ui_state.panel_width - io.MouseDelta.x, panel_min, panel_max);
            }
            ImU32 grip_color =
                grip_active ? IM_COL32(120, 200, 255, 220) : (grip_hovered ? IM_COL32(120, 180, 240, 180) : IM_COL32(80, 110, 150, 120));
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
            bool has_selected_obstacle = ui_state.user_obstacles.selected_index >= 0 &&
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
        if (cfg.initial_pose.enable) {
            if (ImGui::Button("加载初始位姿")) {
                InitialPoseApplyResult result = applyConfiguredInitialPose();
                UiSemanticLevel level         = result.missing_joint_count > 0 ? UiSemanticLevel::Warning : UiSemanticLevel::Success;
                ui_feedback.Push(level, std::string("初始位姿加载: ") + result.detail, now_sec, 4.0);
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
                                 now_sec);
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

            auto rosLevel = ros_bridge.enabled() ? UiSemanticLevel::Success : UiSemanticLevel::Warning;
            ImGui::BeginChild("##top_status_chips", ImVec2(0.0f, 34.0f), false,
                              ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
            KinematicUiFeedback::RenderStatusChip(ros_bridge.enabled() ? "ROS ON" : "ROS OFF", rosLevel);
            ImGui::SameLine();
            KinematicUiFeedback::RenderStatusChip(ik_state.solve_mode == "full_body" ? "IK FULL_BODY" : "IK SINGLE", UiSemanticLevel::Info);
            ImGui::SameLine();
            KinematicUiFeedback::RenderStatusChip(playbackLabel, playbackLevel);
            ImGui::SameLine();
            KinematicUiFeedback::RenderStatusChip(collisionLabel.c_str(), collisionLevel);
            ImGui::EndChild();
        }
        if (ImGui::CollapsingHeader("操作提示")) {
            ImGui::TextDisabled("视角：左键旋转，中键/Shift+左键平移，右键缩放，滚轮缩放");
            ImGui::TextDisabled("快捷键 1-8 切换子页；3D 左键点选 link");
        }
        ImGui::Separator();
        struct SidebarTab {
            const char* label;
            int index;
        };
        const SidebarTab sidebar_tabs[] = {
            {"场景", 0}, {"IK", 1}, {"回放", 2}, {"安全", 3}, {"关节", 4}, {"TF", 5}, {"障碍", 6}, {"规划", 7},
        };
        float avail_w = ImGui::GetContentRegionAvail().x;
        float used_w  = 0.0f;
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(5.0f, 3.0f));
        for (size_t i = 0; i < sizeof(sidebar_tabs) / sizeof(sidebar_tabs[0]); ++i) {
            if (i > 0) {
                const float spacing = ImGui::GetStyle().ItemSpacing.x;
                const float btn_w   = ImGui::CalcTextSize(sidebar_tabs[i].label).x + ImGui::GetStyle().FramePadding.x * 2.0f;
                if (used_w + spacing + btn_w <= avail_w) {
                    ImGui::SameLine();
                    used_w += spacing;
                } else {
                    used_w = 0.0f;
                }
            }
            const bool selected = (ui_state.sidebar_page == sidebar_tabs[i].index);
            if (selected) {
                ImGui::PushStyleColor(ImGuiCol_Button, IM_COL32(90, 155, 235, 255));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, IM_COL32(100, 168, 252, 255));
                ImGui::PushStyleColor(ImGuiCol_ButtonActive, IM_COL32(70, 132, 208, 255));
            }
            if (ImGui::Button(sidebar_tabs[i].label)) {
                ui_state.sidebar_page = sidebar_tabs[i].index;
            }
            used_w += ImGui::GetItemRectSize().x;
            if (selected) {
                ImGui::PopStyleColor(3);
            }
        }
        ImGui::PopStyleVar();
        ImGui::Separator();

        kinematic_viewer::BeginSidebarScrollRegion("##sidebar_scroll");

        if (kinematic_viewer::SidebarPageShowsLinkInspector(ui_state.sidebar_page)) {
            RenderLinkInspectorPanel(&ui_state, &scene, &camera, &collision_state, &collision_result, &playback_state,
                                     &collision_monitor, &link_kinematics_analyzer);
        }

        if (ui_state.sidebar_page == 0) {
            RenderScenePanel(&ui_state);
            RenderRobotTreePanel(&ui_state, &scene);
        }

        auto joints = scene.getJointInfos();
        kinematic_viewer::TickTrajectorySequence(&playback_state, &playback_sm, was_playback_playing, joints, &trajectory_player,
                                                   &scene);
        if (ui_state.sidebar_page == 4) {
            RenderJointPanel(&ui_state, &scene, joints);
        }

        if (ui_state.sidebar_page == 1) {
            RenderIkPanel(&ik_state, &ik_controller, &ros_bridge, &scene);
        }

        if (ui_state.sidebar_page == 2) {
            RenderPlaybackPanel(&playback_state, &trajectory_player, &playback_sm, &scene, joints);
        }

        if (ui_state.sidebar_page == 3) {
            RenderSafetyPanel(&collision_state, collision_result);
        }

        if (ui_state.sidebar_page == 5) {
            RenderTfPanel(&ui_state, scene.getLinkTfInfos());
        }
        if (ui_state.sidebar_page == 6) {
            RenderObstaclePanel(&ui_state);
        }
        if (ui_state.sidebar_page == 7) {
            RenderPathPlannerPanel(&path_planner_ui, &playback_state, &scene, &ik_state.solver, ik_state.chains);
        }

        kinematic_viewer::EndSidebarScrollRegion();

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
                ui_feedback.Push(level, playback_state.trajectory_io_status, now_sec, level == UiSemanticLevel::Error ? 4.5 : 2.8);
            }
            last_playback_io_status = playback_state.trajectory_io_status;
        }

        ImGui::End();
        ui_feedback.RenderToasts(now_sec);

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        app.SwapBuffers();
    }

    // Persist trajectory file list to config before exit
    cfg.playback.trajectory_files.clear();
    for (const auto& entry : playback_state.trajectory_files) {
        cfg.playback.trajectory_files.push_back(entry.path);
    }
    cfg.playback.selected_index = playback_state.selected_trajectory_index;

    DestroyUserObstacleGpuMeshes(&obstacle_meshes);
    glDeleteProgram(mesh_shader);
    glDeleteProgram(line_shader);
    ros_bridge.shutdown();
    return 0;
}
