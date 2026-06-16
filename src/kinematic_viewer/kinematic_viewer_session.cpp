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

struct KinematicViewerSession::Impl {
    LaunchConfig launch;
    KinematicViewerConfig cfg;
    std::string urdf_path;
    KinematicApp* app = nullptr;
    GLFWwindow* window = nullptr;
    int ui_theme_index = 0;
    ConfigWatcher config_watcher{""};

    GLuint mesh_shader = 0;
    GLuint line_shader = 0;
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
    int collision_refresh_tick = 0;
    bool obstacle_gizmo_was_using = false;
    bool obstacle_gizmo_was_over = false;
    bool base_gizmo_was_using = false;
    KinematicInputHandler input_handler;
    double last_frame_sec = 0.0;

    Impl() : playback_sm(&playback_state), ik_controller(&ik_state) {}

    void ShutdownGpu();
    void PersistConfigOnExit();
    InitialPoseApplyResult ApplyConfiguredInitialPose();
    void RunCollisionMonitor();
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
    impl_->launch = launch;
    impl_->cfg = launch.config;
    impl_->urdf_path = launch.urdfPath.empty() ? impl_->cfg.robot.urdf_path : launch.urdfPath;
    impl_->app = app;
    impl_->window = app->Window();
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

    impl_->ui_state.lock_base                  = impl_->cfg.ui.fix_base_like_mujoco;
    impl_->ui_state.mobile_base_drag_available = impl_->cfg.ui.enable_mobile_base_drag && session_internal::IsMobileBaseDragRobot(impl_->urdf_path, impl_->cfg.ui.mobile_base_robots);
    impl_->ui_state.mobile_base_drag_enabled   = impl_->ui_state.mobile_base_drag_available;
    auto appendJointInputGroup          = [&](const std::string& name, const std::vector<std::string>& joint_names) {
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
        std::snprintf(impl_->point_cloud_state.file_path, sizeof(impl_->point_cloud_state.file_path), "%s", impl_->cfg.point_cloud.file_path.c_str());
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
            std::cerr << "[robot_kinematic_viewer] sidebar panels: loaded " << impl_->panel_registry.Count() << "/"
                      << specs.size() << ". Missing plugins are usually stale librkv_panel_*.so — run a full build.\n";
        }
    }
    {
        const int preferred = impl_->panel_registry.IndexOf("joint");
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
        std::snprintf(impl_->teach_state.program_browser_dir, sizeof(impl_->teach_state.program_browser_dir), "%s", teach_browser_dir.c_str());
    }
    if (impl_->teach_state.selected_program_index >= 0) {
        std::string teach_load_error;
        auto& selected_entry = impl_->teach_state.program_files[static_cast<size_t>(impl_->teach_state.selected_program_index)];
        if (LoadTeachProgramFromYaml(impl_->teach_state.program_file_path, &impl_->teach_state, &teach_load_error)) {
            selected_entry.status = "加载成功";
            selected_entry.loaded = true;
            impl_->teach_state.io_status = "启动已加载: " + impl_->teach_state.program_name;
        } else {
            selected_entry.status = "加载失败: " + teach_load_error;
            selected_entry.loaded = false;
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
        std::transform(impl_->ik_state.full_body_backend.begin(), impl_->ik_state.full_body_backend.end(), impl_->ik_state.full_body_backend.begin(),
                       [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
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
            impl_->ik_state.selected_chain = std::clamp(impl_->ik_state.selected_chain, 0, static_cast<int>(impl_->ik_state.chains.size()) - 1);
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
        const double now_sec = glfwGetTime();
        const double dt_sec = std::max(0.0, now_sec - impl_->last_frame_sec);
        impl_->last_frame_sec = now_sec;
        double mouse_x = 0.0;
        double mouse_y = 0.0;
        glfwGetCursorPos(impl_->window, &mouse_x, &mouse_y);
        TickFrame(app, dt_sec, now_sec, mouse_x, mouse_y);
    }
    impl_->PersistConfigOnExit();
}

void KinematicViewerSession::TickFrame(KinematicApp& app, double dt_sec, double now_sec, double mouse_x, double mouse_y) {
        // --- Deferred initial pose ---
        if (impl_->initial_pose_auto_apply_pending) {
            InitialPoseApplyResult result = impl_->ApplyConfiguredInitialPose();
            UiSemanticLevel level         = result.missing_joint_count > 0 ? UiSemanticLevel::Warning : UiSemanticLevel::Success;
            impl_->ui_feedback.Push(level, std::string("初始位姿加载: ") + result.detail, now_sec, 4.0);
            impl_->initial_pose_auto_apply_pending = false;
        }

        int fb_w = 0, fb_h = 0;
        glfwGetFramebufferSize(impl_->window, &fb_w, &fb_h);
        float panel_min      = 280.0f;
        float panel_max      = std::max(panel_min, static_cast<float>(fb_w) - 320.0f);
        impl_->ui_state.panel_width = std::clamp(impl_->ui_state.panel_width, panel_min, panel_max);
        const int panel_w    = impl_->ui_state.sidebar_hidden ? 0 : static_cast<int>(impl_->ui_state.panel_width);
        int viewport_w       = std::max(1, fb_w - panel_w);
        int viewport_h       = std::max(1, fb_h);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        ImGuizmo::BeginFrame();

        // --- Global hotkeys (sidebar / playback) ---
        const bool sidebar_hotkeys_enabled = !ImGui::GetIO().WantTextInput && !ImGui::GetIO().WantCaptureKeyboard;
        impl_->ui_state.sidebar_page =
            impl_->input_handler.HandleSidebarHotkeys(impl_->ui_state.sidebar_page, impl_->panel_registry.Count(), sidebar_hotkeys_enabled);

        const auto viewport_hotkeys = impl_->input_handler.HandleViewportHotkeys(
            sidebar_hotkeys_enabled, impl_->playback_sm.HasKeyframes(), static_cast<float>(dt_sec), impl_->playback_state.play_speed);
        if (viewport_hotkeys.toggled_sidebar) {
            impl_->ui_state.sidebar_hidden = !impl_->ui_state.sidebar_hidden;
            impl_->ui_feedback.Push(UiSemanticLevel::Info, impl_->ui_state.sidebar_hidden ? "已隐藏侧栏 (H 恢复)" : "已显示侧栏", now_sec);
        }
        if (viewport_hotkeys.toggled_playback) {
            if (impl_->playback_sm.TogglePlayPause()) {
                impl_->ui_feedback.Push(UiSemanticLevel::Info, impl_->playback_sm.IsPlaying() ? "回放: 播放" : "回放: 暂停", now_sec, 1.2);
            }
        }
        if (viewport_hotkeys.playback_speed_adjust != 0) {
            const float prev_speed = impl_->playback_state.play_speed;
            impl_->playback_state.play_speed =
                std::clamp(impl_->playback_state.play_speed + viewport_hotkeys.playback_speed_adjust * 0.1f, 0.1f, 3.0f);
            if (impl_->playback_state.play_speed != prev_speed) {
                char speed_msg[64];
                std::snprintf(speed_msg, sizeof(speed_msg), "回放倍速: %.2fx", impl_->playback_state.play_speed);
                impl_->ui_feedback.Push(UiSemanticLevel::Info, speed_msg, now_sec, 0.8);
            }
        }
        if (viewport_hotkeys.playback_step_count > 0 && viewport_hotkeys.playback_step_direction != 0) {
            for (int step = 0; step < viewport_hotkeys.playback_step_count; ++step) {
                if (impl_->playback_sm.StepKeyframe(viewport_hotkeys.playback_step_direction)) {
                    impl_->trajectory_player.SampleAtCurrentTime(impl_->playback_state, &impl_->scene);
                }
            }
        }
        if (impl_->panel_registry.Count() == 0) {
            impl_->ui_state.sidebar_page = 0;
        } else {
            impl_->ui_state.sidebar_page = std::clamp(impl_->ui_state.sidebar_page, 0, impl_->panel_registry.Count() - 1);
        }
        const std::string current_panel_key =
            impl_->panel_registry.Count() == 0 ? std::string("scene") : impl_->panel_registry.Id(impl_->ui_state.sidebar_page);
        impl_->ui_state.scene_panel_active = (current_panel_key == "scene");

        // --- Viewport input: hover, joint drag, link pick ---
        KinematicInputHandler::UpdateContext input_ctx;
        input_ctx.mouse_x             = mouse_x;
        input_ctx.mouse_y             = mouse_y;
        input_ctx.viewport_w          = viewport_w;
        input_ctx.viewport_h          = viewport_h;
        input_ctx.imgui_wants_mouse   = ImGui::GetIO().WantCaptureMouse;
        input_ctx.panel_resize_active = impl_->ui_state.panel_resize_active;
        input_ctx.ik_gizmo_using      = impl_->ik_state.gizmo_was_using;
        input_ctx.ik_gizmo_over       = impl_->ik_state.gizmo_was_over;
        input_ctx.obs_gizmo_using     = impl_->obstacle_gizmo_was_using || impl_->base_gizmo_was_using;
        input_ctx.obs_gizmo_over      = impl_->obstacle_gizmo_was_over;
        input_ctx.ik_dragging_marker  = impl_->ik_state.dragging_marker;
        input_ctx.sidebar_page        = (current_panel_key == "scene") ? 0 : ((current_panel_key == "obstacle") ? 6 : -1);
        input_ctx.scroll_delta        = GetScrollDelta();
        input_ctx.left_mouse_down     = glfwGetMouseButton(impl_->window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
        input_ctx.middle_mouse_down   = glfwGetMouseButton(impl_->window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
        input_ctx.right_mouse_down    = glfwGetMouseButton(impl_->window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
        input_ctx.shift_key_down =
            glfwGetKey(impl_->window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || glfwGetKey(impl_->window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
        input_ctx.enable_joint_drag = impl_->ui_state.enable_joint_drag_rotation;

        const glm::mat4 pick_proj =
            glm::perspective(glm::radians(50.0f), static_cast<float>(viewport_w) / static_cast<float>(viewport_h), 0.05f, 80.0f);
        const glm::mat4 pick_view = impl_->camera.viewMatrix();

        const bool hover_for_joint_drag =
            impl_->ui_state.enable_joint_drag_rotation && !input_ctx.ik_gizmo_using && !input_ctx.obs_gizmo_using && !input_ctx.imgui_wants_mouse;
        if (hover_for_joint_drag || impl_->ui_state.enable_link_hover_highlight) {
            auto link_hover = impl_->input_handler.UpdateLinkHover(input_ctx, pick_view, pick_proj, &impl_->scene, glfwGetTime(), hover_for_joint_drag);
            if (!link_hover.throttle_skip) {
                impl_->ui_state.hovered_link = link_hover.picked ? link_hover.link_name : std::string();
            }
        } else if (!impl_->input_handler.IsJointDragActive()) {
            impl_->ui_state.hovered_link.clear();
        }

        input_ctx.hovered_link = &impl_->ui_state.hovered_link;
        if (!impl_->ui_state.hovered_link.empty()) {
            std::string parent_joint;
            input_ctx.hovered_link_draggable =
                impl_->scene.getParentJointNameForLink(impl_->ui_state.hovered_link, &parent_joint) && [&]() {
                    rkv::RobotScene::JointInfo joint_info;
                    return impl_->scene.getJointInfo(parent_joint, &joint_info) && joint_info.revolute;
                }();
        } else {
            input_ctx.hovered_link_draggable = false;
        }

        auto joint_drag = impl_->input_handler.UpdateJointDrag(input_ctx, pick_view, pick_proj, &impl_->scene);
        if (joint_drag.started) {
            impl_->ui_state.selected_link            = joint_drag.link_name;
            impl_->ui_state.trajectory_min_surface_m = -1.0f;
            const auto joints                 = impl_->scene.getJointInfos();
            impl_->ui_state.selected_joint           = -1;
            for (int i = 0; i < static_cast<int>(joints.size()); ++i) {
                if (joints[static_cast<size_t>(i)].name == joint_drag.joint_name) {
                    impl_->ui_state.selected_joint = i;
                    break;
                }
            }
        } else if (joint_drag.dragging) {
            impl_->ui_state.selected_link = joint_drag.link_name;
        }

        const bool joint_pose_dirty_before_playback = impl_->scene.isJointPoseDirty();
        if (joint_pose_dirty_before_playback) {
            impl_->scene.updateTransforms();
        }

        // --- Playback advance & collision ---
        const bool was_playback_playing = impl_->playback_sm.IsPlaying();
        impl_->playback_sm.AdvanceTime(static_cast<float>(dt_sec));
        if (impl_->playback_sm.IsPlaying()) {
            impl_->trajectory_player.SampleAtCurrentTime(impl_->playback_state, &impl_->scene);
        }
        impl_->scene.setFixedBaseMode(impl_->ui_state.lock_base);
        impl_->scene.updateTransforms();

        if (impl_->collision_state.enable && !impl_->input_handler.IsJointDragActive() &&
            (impl_->scene.isJointPoseDirty() || impl_->playback_sm.IsPlaying() || (++impl_->collision_refresh_tick % 4 == 0))) {
            impl_->RunCollisionMonitor();
        }

        glm::mat4 proj =
            glm::perspective(glm::radians(50.0f), static_cast<float>(viewport_w) / static_cast<float>(viewport_h), 0.05f, 80.0f);
        glm::mat4 view = impl_->camera.viewMatrix();

        // --- 3D viewport render ---
        KinematicRenderLoop::Context render_ctx;
        render_ctx.viewport_w        = viewport_w;
        render_ctx.viewport_h        = viewport_h;
        render_ctx.mesh_shader       = impl_->mesh_shader;
        render_ctx.line_shader       = impl_->line_shader;
        render_ctx.point_shader      = impl_->point_shader;
        render_ctx.point_cloud       = &impl_->point_cloud_state;
        render_ctx.point_cloud_layer = &impl_->point_cloud_layer;
        render_ctx.scene             = &impl_->scene;
        render_ctx.ui_state          = &impl_->ui_state;
        render_ctx.ik_state          = &impl_->ik_state;
        render_ctx.collision_state   = &impl_->collision_state;
        render_ctx.collision_result  = &impl_->collision_result;
        render_ctx.obstacle_meshes   = &impl_->obstacle_meshes;
        render_ctx.camera            = &impl_->camera;
        render_ctx.planned_path      = &impl_->path_planner_ui.preview_waypoints;
        render_ctx.show_planned_path = impl_->path_planner_ui.show_preview;
        render_ctx.demo_visual_mode  = impl_->ui_state.demo_visual_mode;
        impl_->render_loop.Render(render_ctx);

        kinematic_viewer::CaptureFrameForRecorder(&impl_->video_recorder, viewport_w, viewport_h);

        auto applyIkForActiveChain = [&](bool force_orientation_lock, bool fast_mode, bool prefer_position_only_target) -> bool {
            return impl_->ik_controller.ApplyIkForActiveChain(&impl_->scene, force_orientation_lock, fast_mode, prefer_position_only_target);
        };

        auto refineActiveChainToMarker = [&]() -> bool {
            return impl_->ik_controller.RefineActiveChainToMarker(&impl_->scene);
        };

        auto activeChainPositionErrorMmToMarker = [&]() -> float {
            return impl_->ik_controller.ActiveChainPositionErrorMmToMarker(&impl_->scene);
        };

        // External target path disabled; keep local marker update path.
        impl_->ik_controller.ApplyExternalTarget(&impl_->scene);

        // --- ImGuizmo: obstacle / base / IK marker ---
        // RViz-like manipulator via ImGuizmo
        // Draw gizmo directly on the foreground drawlist of the 3D viewport area.
        ImGuizmo::SetOrthographic(false);
        ImGuizmo::AllowAxisFlip(false);
        ImGuizmo::SetDrawlist(ImGui::GetForegroundDrawList());
        ImGuizmo::SetRect(0.0f, 0.0f, static_cast<float>(viewport_w), static_cast<float>(viewport_h));

        const bool obstacle_page_active = (current_panel_key == "scene" || current_panel_key == "obstacle");
        const bool obstacle_edit_active = obstacle_page_active && impl_->ui_state.user_obstacles.enable_pose_gizmo &&
                                          impl_->ui_state.user_obstacles.selected_index >= 0 &&
                                          impl_->ui_state.user_obstacles.selected_index < static_cast<int>(impl_->ui_state.user_obstacles.items.size());
        if (obstacle_edit_active) {
            auto& obs = impl_->ui_state.user_obstacles.items[static_cast<size_t>(impl_->ui_state.user_obstacles.selected_index)];
            ImGuizmo::SetGizmoSizeClipSpace(impl_->ui_state.user_obstacles.gizmo_size_clip_space);
            glm::mat4 obs_world        = markerWorldMatrix(obs.position, obs.rpy_deg);
            ImGuizmo::OPERATION obs_op = static_cast<ImGuizmo::OPERATION>(ImGuizmo::TRANSLATE | ImGuizmo::ROTATE);
            if (impl_->ui_state.user_obstacles.gizmo_operation == 0) {
                obs_op = ImGuizmo::TRANSLATE;
            } else if (impl_->ui_state.user_obstacles.gizmo_operation == 1) {
                obs_op = ImGuizmo::ROTATE;
            }
            ImGuizmo::MODE obs_mode = impl_->ui_state.user_obstacles.gizmo_world_mode ? ImGuizmo::WORLD : ImGuizmo::LOCAL;
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
            impl_->obstacle_gizmo_was_using = obs_using;
            impl_->obstacle_gizmo_was_over  = obs_over;
        } else {
            impl_->obstacle_gizmo_was_using = false;
            impl_->obstacle_gizmo_was_over  = false;
        }

        const bool base_edit_active =
            impl_->ui_state.mobile_base_drag_available && impl_->ui_state.mobile_base_drag_enabled && current_panel_key == "scene";
        if (base_edit_active && !obstacle_edit_active && !impl_->ik_state.gizmo_was_using) {
            float base_x_m = 0.0f;
            float base_y_m = 0.0f;
            float base_yaw = 0.0f;
            if (impl_->scene.getVirtualBasePose2D(&base_x_m, &base_y_m, &base_yaw)) {
                ImGuizmo::SetGizmoSizeClipSpace(0.12f);
                glm::mat4 base_world =
                    markerWorldMatrix(glm::vec3(base_x_m, base_y_m, 0.0f), glm::vec3(0.0f, 0.0f, glm::degrees(base_yaw)));
                ImGuizmo::OPERATION base_op = static_cast<ImGuizmo::OPERATION>(ImGuizmo::TRANSLATE | ImGuizmo::ROTATE);
                if (impl_->ui_state.mobile_base_gizmo_operation == 0) {
                    base_op = ImGuizmo::TRANSLATE;
                } else if (impl_->ui_state.mobile_base_gizmo_operation == 1) {
                    base_op = ImGuizmo::ROTATE;
                }
                glm::mat4 base_delta(1.0f);
                bool base_manipulated = ImGuizmo::Manipulate(glm::value_ptr(view), glm::value_ptr(proj), base_op, ImGuizmo::WORLD,
                                                             glm::value_ptr(base_world), glm::value_ptr(base_delta), nullptr);
                bool base_using       = ImGuizmo::IsUsing();
                if (base_manipulated || base_using) {
                    const glm::vec3 raw_pos = glm::vec3(base_world[3]);
                    const glm::mat3 rot_mat(base_world);
                    // Planar yaw about world Z (do not use glm::eulerAngles — unstable for gizmo output).
                    const float yaw_rad = std::atan2(rot_mat[0][1], rot_mat[0][0]);
                    float new_x         = raw_pos.x;
                    float new_y         = raw_pos.y;
                    float new_yaw       = yaw_rad;
                    if (impl_->ui_state.mobile_base_gizmo_operation == 0) {
                        new_yaw = base_yaw;
                    } else if (impl_->ui_state.mobile_base_gizmo_operation == 1) {
                        new_x = base_x_m;
                        new_y = base_y_m;
                    }
                    impl_->scene.setVirtualBasePose2D(new_x, new_y, new_yaw);
                }
                impl_->base_gizmo_was_using = base_using;
            } else {
                impl_->base_gizmo_was_using = false;
            }
        } else {
            impl_->base_gizmo_was_using = false;
        }

        ImGuizmo::SetGizmoSizeClipSpace(impl_->ik_state.gizmo_size_clip_space);
        const bool ik_page_active = (current_panel_key == "ik");
        if (ik_page_active && !obstacle_edit_active && !impl_->base_gizmo_was_using && impl_->ik_state.selected_chain >= 0 &&
            impl_->ik_state.selected_chain < static_cast<int>(impl_->ik_state.chains.size())) {
            if (!impl_->ik_state.marker_initialized) {
                impl_->ik_controller.LoadActiveMarkerFromTarget(&impl_->scene);
            }

            const glm::vec3 marker_pos(impl_->ik_state.marker_pos[0], impl_->ik_state.marker_pos[1], impl_->ik_state.marker_pos[2]);
            const glm::vec3 marker_rpy_deg(impl_->ik_state.marker_rpy_deg[0], impl_->ik_state.marker_rpy_deg[1], impl_->ik_state.marker_rpy_deg[2]);
            const glm::mat4 marker_world_before = markerWorldMatrix(marker_pos, marker_rpy_deg);
            glm::mat4 gizmo_world               = marker_world_before;
            ImGuizmo::OPERATION op              = static_cast<ImGuizmo::OPERATION>(ImGuizmo::TRANSLATE | ImGuizmo::ROTATE);
            if (impl_->ik_state.gizmo_operation == 0)
                op = ImGuizmo::TRANSLATE;
            if (impl_->ik_state.gizmo_operation == 1)
                op = ImGuizmo::ROTATE;
            ImGuizmo::MODE mode = impl_->ik_state.gizmo_world_mode ? ImGuizmo::WORLD : ImGuizmo::LOCAL;

            float snap_values[3] = {0.0f, 0.0f, 0.0f};
            float* snap_ptr      = nullptr;
            if (op == ImGuizmo::TRANSLATE && impl_->ik_state.translate_snap_enabled) {
                snap_values[0] = impl_->ik_state.translate_snap_step_m;
                snap_values[1] = impl_->ik_state.translate_snap_step_m;
                snap_values[2] = impl_->ik_state.translate_snap_step_m;
                snap_ptr       = snap_values;
            } else if (op == ImGuizmo::ROTATE && impl_->ik_state.rotate_snap_enabled) {
                snap_values[0] = impl_->ik_state.rotate_snap_step_deg;
                snap_values[1] = impl_->ik_state.rotate_snap_step_deg;
                snap_values[2] = impl_->ik_state.rotate_snap_step_deg;
                snap_ptr       = snap_values;
            }

            glm::mat4 gizmo_delta(1.0f);
            bool manipulated = ImGuizmo::Manipulate(glm::value_ptr(view), glm::value_ptr(proj), op, mode, glm::value_ptr(gizmo_world),
                                                    glm::value_ptr(gizmo_delta), snap_ptr);
            bool gizmo_using = ImGuizmo::IsUsing();
            bool gizmo_over  = ImGuizmo::IsOver();
            bool current_drag_position_only = (op == ImGuizmo::TRANSLATE);
            if (manipulated || gizmo_using) {
                impl_->ik_state.dragging_marker       = true;
                impl_->ik_state.gizmo_drag_interacted = true;

                const glm::vec3 raw_pos = glm::vec3(gizmo_world[3]);
                const glm::quat raw_q   = glm::quat_cast(gizmo_world);
                const glm::vec3 raw_rpy_deg(glm::degrees(glm::eulerAngles(raw_q)));

                const glm::vec3 delta_pos = raw_pos - marker_pos;
                glm::vec3 scaled_delta_pos(delta_pos.x * impl_->ik_state.translate_channel_gain[0],
                                           delta_pos.y * impl_->ik_state.translate_channel_gain[1],
                                           delta_pos.z * impl_->ik_state.translate_channel_gain[2]);

                const glm::vec3 raw_delta_rpy_deg(wrapDeltaDeg(raw_rpy_deg.x - marker_rpy_deg.x),
                                                  wrapDeltaDeg(raw_rpy_deg.y - marker_rpy_deg.y),
                                                  wrapDeltaDeg(raw_rpy_deg.z - marker_rpy_deg.z));
                glm::vec3 scaled_delta_rpy_deg(raw_delta_rpy_deg.x * impl_->ik_state.rotate_channel_gain[0],
                                               raw_delta_rpy_deg.y * impl_->ik_state.rotate_channel_gain[1],
                                               raw_delta_rpy_deg.z * impl_->ik_state.rotate_channel_gain[2]);

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
                impl_->ik_state.gizmo_drag_position_only = current_drag_position_only;

                impl_->ik_state.marker_pos[0] = updated_pos.x;
                impl_->ik_state.marker_pos[1] = updated_pos.y;
                impl_->ik_state.marker_pos[2] = updated_pos.z;
                if (!impl_->ik_state.lock_orientation) {
                    impl_->ik_state.marker_rpy_deg[0] = updated_rpy_deg.x;
                    impl_->ik_state.marker_rpy_deg[1] = updated_rpy_deg.y;
                    impl_->ik_state.marker_rpy_deg[2] = updated_rpy_deg.z;
                }
                impl_->ik_controller.SaveActiveMarkerToTarget();
                impl_->ik_state.gizmo_pose_dirty = true;
            } else {
                impl_->ik_state.dragging_marker = false;
            }

            // Realtime IK updates while dragging (RViz-like immediate feedback), throttled by frequency.
            if (gizmo_using && impl_->ik_state.gizmo_pose_dirty && impl_->ik_state.realtime_ik_during_drag) {
                const float effective_hz  = std::max(5.0f, impl_->ik_state.realtime_ik_hz);
                const double interval_sec = 1.0 / static_cast<double>(effective_hz);
                if (impl_->ik_state.last_realtime_ik_apply_sec < 0.0 || (now_sec - impl_->ik_state.last_realtime_ik_apply_sec) >= interval_sec) {
                    applyIkForActiveChain(false, true, current_drag_position_only);
                    impl_->ik_state.last_realtime_ik_apply_sec = now_sec;
                    impl_->ik_state.gizmo_pose_dirty           = false;
                }
            }

            // Sync IK only when drag ends (mouse release / gizmo released)
            if (!gizmo_using && impl_->ik_state.gizmo_was_using && impl_->ik_state.gizmo_drag_interacted) {
                applyIkForActiveChain(false, false, impl_->ik_state.gizmo_drag_position_only);
                const bool drag_had_rotation               = !impl_->ik_state.gizmo_drag_position_only;
                const bool should_refine_with_single_chain = impl_->ik_state.solve_mode == "full_body" && impl_->ui_state.lock_base &&
                                                             impl_->ik_state.refine_single_chain_on_drag_end &&
                                                             (!impl_->ik_state.refine_only_when_rotation || drag_had_rotation);
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
                impl_->ik_state.gizmo_pose_dirty         = false;
                impl_->ik_state.gizmo_drag_interacted    = false;
                impl_->ik_state.gizmo_drag_position_only = true;
            }
            impl_->ik_state.gizmo_was_using = gizmo_using;
            impl_->ik_state.gizmo_was_over  = gizmo_over;
        } else {
            impl_->ik_state.gizmo_was_using = false;
            impl_->ik_state.gizmo_was_over  = false;
        }

        // --- Viewport picking & camera ---
        // Click in 3D viewport to select nearest obstacle.
        auto obstacle_pick = impl_->input_handler.UpdateObstaclePick(input_ctx, pick_view, pick_proj, impl_->ui_state.user_obstacles);
        if (obstacle_pick.picked) {
            impl_->ui_state.user_obstacles.selected_index = obstacle_pick.selected_index;
        }

        if (!hover_for_joint_drag && impl_->ui_state.enable_link_hover_highlight) {
            auto link_hover = impl_->input_handler.UpdateLinkHover(input_ctx, pick_view, pick_proj, &impl_->scene, glfwGetTime(), false);
            if (!link_hover.throttle_skip) {
                impl_->ui_state.hovered_link = link_hover.picked ? link_hover.link_name : std::string();
            }
        } else if (!impl_->ui_state.enable_link_hover_highlight && !impl_->ui_state.enable_joint_drag_rotation) {
            impl_->ui_state.hovered_link.clear();
        }

        if (impl_->ui_state.enable_link_click_select) {
            auto link_pick = impl_->input_handler.UpdateLinkPick(input_ctx, pick_view, pick_proj, &impl_->scene);
            if (link_pick.picked) {
                impl_->ui_state.selected_link            = link_pick.link_name;
                impl_->ui_state.trajectory_min_surface_m = -1.0f;
                impl_->ui_state.selected_joint           = -1;
            }
        }

        input_ctx.ik_gizmo_using     = impl_->ik_state.gizmo_was_using;
        input_ctx.ik_dragging_marker = impl_->ik_state.dragging_marker;
        input_ctx.obs_gizmo_using    = impl_->obstacle_gizmo_was_using || impl_->base_gizmo_was_using;
        input_ctx.joint_drag_active       = impl_->input_handler.IsJointDragActive();
        input_ctx.hovered_link_draggable  = input_ctx.hovered_link_draggable || impl_->input_handler.IsJointDragActive();
        input_ctx.imgui_wants_mouse       = ImGui::GetIO().WantCaptureMouse;
        impl_->input_handler.UpdateCamera(&impl_->camera, input_ctx);

        // --- Viewport HUD (playback bar, overlays) ---
        {
            kinematic_viewer::ViewportHudContext hud_ctx;
            hud_ctx.viewport_w        = viewport_w;
            hud_ctx.viewport_h        = viewport_h;
            hud_ctx.ui_state          = &impl_->ui_state;
            hud_ctx.playback_state    = &impl_->playback_state;
            hud_ctx.playback_sm       = &impl_->playback_sm;
            hud_ctx.playback_player   = &impl_->trajectory_player;
            hud_ctx.scene             = &impl_->scene;
            hud_ctx.collision_state   = &impl_->collision_state;
            hud_ctx.collision_result  = &impl_->collision_result;
            hud_ctx.video_recorder    = &impl_->video_recorder;
            kinematic_viewer::RenderViewportHud(hud_ctx);
        }

        // --- Sidebar panels ---
        if (panel_w > 0) {
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
            impl_->ui_state.panel_resize_active = grip_active;
            if (grip_hovered || grip_active) {
                ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
            }
            if (grip_active) {
                impl_->ui_state.panel_width = std::clamp(impl_->ui_state.panel_width - io.MouseDelta.x, panel_min, panel_max);
            }
            ImU32 grip_color =
                grip_active ? IM_COL32(120, 200, 255, 220) : (grip_hovered ? IM_COL32(120, 180, 240, 180) : IM_COL32(80, 110, 150, 120));
            ImGui::GetWindowDrawList()->AddRectFilled(grip_min, grip_max, grip_color, 2.0f);
            ImGui::SetCursorPosY(8.0f);
        }

        {
            const std::string urdf_name = kinematic_viewer::PathBasename(impl_->urdf_path);
            ImGui::Text("URDF: %s", urdf_name.c_str());
            if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
                ImGui::SetTooltip("%s", impl_->urdf_path.c_str());
            }
        }
        if (ImGui::CollapsingHeader("快捷操作")) {
            if (ImGui::SmallButton("重置视角")) {
                impl_->camera.distance = impl_->cfg.camera.distance;
                impl_->camera.yaw      = impl_->cfg.camera.yaw;
                impl_->camera.pitch    = impl_->cfg.camera.pitch;
                impl_->camera.target   = impl_->cfg.camera.target;
            }
            ImGui::SameLine();
            if (ImGui::SmallButton("对准Marker")) {
                impl_->camera.target = glm::vec3(impl_->ik_state.marker_pos[0], impl_->ik_state.marker_pos[1], impl_->ik_state.marker_pos[2]);
            }
            ImGui::SameLine();
            bool has_selected_obstacle = impl_->ui_state.user_obstacles.selected_index >= 0 &&
                                         impl_->ui_state.user_obstacles.selected_index < static_cast<int>(impl_->ui_state.user_obstacles.items.size());
            if (!has_selected_obstacle) {
                ImGui::BeginDisabled();
            }
            if (ImGui::SmallButton("对准障碍")) {
                const auto& obs = impl_->ui_state.user_obstacles.items[static_cast<size_t>(impl_->ui_state.user_obstacles.selected_index)];
                impl_->camera.target   = obs.position;
            }
            if (!has_selected_obstacle) {
                ImGui::EndDisabled();
            }
            ImGui::SameLine();
            const bool has_selected_link = !impl_->ui_state.selected_link.empty();
            if (!has_selected_link) {
                ImGui::BeginDisabled();
            }
            if (ImGui::SmallButton("对准Link")) {
                FocusCameraOnLink(&impl_->camera, impl_->scene, impl_->ui_state.selected_link);
            }
            if (!has_selected_link) {
                ImGui::EndDisabled();
            }
        }

        if (ImGui::CollapsingHeader("视频录制")) {
            kinematic_viewer::RenderVideoRecorderPanel(&impl_->ui_state, &impl_->video_recorder, &impl_->ui_feedback, now_sec, viewport_w, viewport_h);
        }
        if (impl_->cfg.initial_pose.enable) {
            if (ImGui::Button("加载初始位姿")) {
                InitialPoseApplyResult result = impl_->ApplyConfiguredInitialPose();
                UiSemanticLevel level         = result.missing_joint_count > 0 ? UiSemanticLevel::Warning : UiSemanticLevel::Success;
                impl_->ui_feedback.Push(level, std::string("初始位姿加载: ") + result.detail, now_sec, 4.0);
            }
            ImGui::SameLine();
            ImGui::TextDisabled("来自 config.initial_pose");
        }
        {
            const auto& theme_names = kinematic_viewer::KinematicUiThemeNames();
            int new_theme_index     = impl_->ui_theme_index;
            kinematic_viewer::PushSidebarFullWidth();
            if (ImGui::Combo("主题", &new_theme_index, theme_names.data(), static_cast<int>(theme_names.size()))) {
                impl_->ui_theme_index = new_theme_index;
                kinematic_viewer::ApplyKinematicUiStyleByIndex(impl_->ui_theme_index);
                impl_->ui_feedback.Push(UiSemanticLevel::Info, std::string("主题切换: ") + theme_names[static_cast<size_t>(impl_->ui_theme_index)],
                                 now_sec);
            }
            kinematic_viewer::PopSidebarWidth();
        }
        {
            auto playbackLevel        = UiSemanticLevel::Info;
            const char* playbackLabel = "回放 STOP";
            if (impl_->playback_sm.IsPlaying()) {
                playbackLevel = UiSemanticLevel::Success;
                playbackLabel = "回放 PLAY";
            } else if (impl_->playback_sm.IsPaused()) {
                playbackLevel = UiSemanticLevel::Warning;
                playbackLabel = "回放 PAUSE";
            }

            auto collisionLevel = UiSemanticLevel::Info;
            std::string collisionLabel("碰撞 --");
            if (impl_->collision_state.has_valid_distance) {
                if (impl_->collision_state.nearest_surface_distance_m <= impl_->collision_state.danger_distance_m) {
                    collisionLevel = UiSemanticLevel::Error;
                    collisionLabel = "碰撞 DANGER";
                } else if (impl_->collision_state.nearest_surface_distance_m <= impl_->collision_state.warning_distance_m) {
                    collisionLevel = UiSemanticLevel::Warning;
                    collisionLabel = "碰撞 WARN";
                } else {
                    collisionLevel = UiSemanticLevel::Success;
                    collisionLabel = "碰撞 SAFE";
                }
            }

            ImGui::BeginChild("##top_status_chips", ImVec2(0.0f, 34.0f), false,
                              ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
            KinematicUiFeedback::RenderStatusChip(impl_->ik_state.solve_mode == "full_body" ? "IK FULL_BODY" : "IK SINGLE", UiSemanticLevel::Info);
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
        for (int i = 0; i < impl_->panel_registry.Count(); ++i) {
            if (i > 0) {
                const float spacing = ImGui::GetStyle().ItemSpacing.x;
                const float btn_w   = ImGui::CalcTextSize(impl_->panel_registry.Label(i).c_str()).x + ImGui::GetStyle().FramePadding.x * 2.0f;
                if (used_w + spacing + btn_w <= avail_w) {
                    ImGui::SameLine();
                    used_w += spacing;
                } else {
                    used_w = 0.0f;
                }
            }
            const bool selected = (impl_->ui_state.sidebar_page == i);
            if (selected) {
                ImGui::PushStyleColor(ImGuiCol_Button, IM_COL32(90, 155, 235, 255));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, IM_COL32(100, 168, 252, 255));
                ImGui::PushStyleColor(ImGuiCol_ButtonActive, IM_COL32(70, 132, 208, 255));
            }
            if (ImGui::Button(impl_->panel_registry.Label(i).c_str())) {
                impl_->ui_state.sidebar_page = i;
            }
            used_w += ImGui::GetItemRectSize().x;
            if (selected) {
                ImGui::PopStyleColor(3);
            }
        }
        ImGui::PopStyleVar();
        ImGui::Separator();

        kinematic_viewer::BeginSidebarScrollRegion("##sidebar_scroll");
        kinematic_viewer::RenderAngleUnitSelector(&impl_->ui_state.angle_unit_deg);
        ImGui::Separator();

        // link inspector is shared between impl_->scene and tf panels.
        if (current_panel_key == "scene" || current_panel_key == "tf") {
            RenderLinkInspectorPanel(&impl_->ui_state, &impl_->scene, &impl_->camera, &impl_->collision_state, &impl_->collision_result, &impl_->playback_state, &impl_->collision_monitor,
                                     &impl_->link_kinematics_analyzer);
        }

        auto joints   = impl_->scene.getJointInfos();
        auto tf_infos = impl_->scene.getLinkTfInfos();
        kinematic_viewer::TickTrajectorySequence(&impl_->playback_state, &impl_->playback_sm, was_playback_playing, joints, &impl_->trajectory_player, &impl_->scene);

        // Build the context bag and dispatch to the active panel plugin.
        RkvPanelCtx panel_ctx{};
        panel_ctx.viewer_state            = &impl_->ui_state;
        panel_ctx.ik_state                = &impl_->ik_state;
        panel_ctx.ik_controller           = &impl_->ik_controller;
        panel_ctx.scene                   = &impl_->scene;
        panel_ctx.camera                  = &impl_->camera;
        panel_ctx.collision_state         = &impl_->collision_state;
        panel_ctx.collision_result        = &impl_->collision_result;
        panel_ctx.collision_monitor       = &impl_->collision_monitor;
        panel_ctx.playback_state          = &impl_->playback_state;
        panel_ctx.playback_player         = &impl_->trajectory_player;
        panel_ctx.playback_sm             = &impl_->playback_sm;
        panel_ctx.teach_state             = &impl_->teach_state;
        panel_ctx.point_cloud_state       = &impl_->point_cloud_state;
        panel_ctx.point_cloud_layer       = &impl_->point_cloud_layer;
        panel_ctx.path_planner_ui         = &impl_->path_planner_ui;
        panel_ctx.link_kinematics_analyzer = &impl_->link_kinematics_analyzer;
        panel_ctx.joints                  = &joints;
        panel_ctx.tf_infos                = &tf_infos;
        panel_ctx.ik_solver               = &impl_->ik_state.solver;
        panel_ctx.ik_chains               = &impl_->ik_state.chains;

        impl_->panel_registry.Render(impl_->ui_state.sidebar_page, &panel_ctx);

        kinematic_viewer::EndSidebarScrollRegion();

        if (impl_->scene.consumeJointPoseDirty()) {
            impl_->scene.updateTransforms();
            impl_->RunCollisionMonitor();
        }

        if (impl_->playback_state.trajectory_io_status != impl_->last_playback_io_status) {
            if (!impl_->playback_state.trajectory_io_status.empty()) {
                UiSemanticLevel level = UiSemanticLevel::Info;
                if (impl_->playback_state.trajectory_io_status.find("失败") != std::string::npos) {
                    level = UiSemanticLevel::Error;
                } else if (impl_->playback_state.trajectory_io_status.find("成功") != std::string::npos) {
                    level = UiSemanticLevel::Success;
                } else if (impl_->playback_state.trajectory_io_status.find("告警") != std::string::npos) {
                    level = UiSemanticLevel::Warning;
                }
                impl_->ui_feedback.Push(level, impl_->playback_state.trajectory_io_status, now_sec, level == UiSemanticLevel::Error ? 4.5 : 2.8);
            }
            impl_->last_playback_io_status = impl_->playback_state.trajectory_io_status;
        }

        ImGui::End();
        }

        impl_->ui_feedback.RenderToasts(now_sec, static_cast<float>(viewport_w) - 14.0f, 14.0f);

        // --- Present ---
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        app.SwapBuffers();
}

}  // namespace kinematic_viewer
