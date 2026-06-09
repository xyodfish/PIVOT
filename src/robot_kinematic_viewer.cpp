#include "kinematic_viewer/kinematic_app.h"
#include "kinematic_viewer/kinematic_bootstrap.h"
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
#include "kinematic_viewer/kinematic_demo_visual.h"
#include "kinematic_viewer/kinematic_viewport_hud.h"
#include "kinematic_viewer/kinematic_runtime_state.h"
#include "kinematic_viewer/kinematic_shader_utils.h"
#include "kinematic_viewer/kinematic_robot_tree_panel.h"
#include "kinematic_viewer/kinematic_angle_units.h"
#include "kinematic_viewer/kinematic_sidebar_layout.h"
#include "kinematic_viewer/kinematic_sidebar_panels.h"
#include "kinematic_viewer/kinematic_string_utils.h"
#include "kinematic_viewer/kinematic_ui_feedback.h"
#include "kinematic_viewer/kinematic_ui_theme.h"
#include "kinematic_viewer/kinematic_user_obstacles.h"
#include "kinematic_viewer/kinematic_video_panel.h"
#include "kinematic_viewer/kinematic_video_recorder.h"
#include "kinematic_viewer/kinematic_teach.h"
#include "kinematic_viewer/kinematic_teach_panel.h"
#include "kinematic_viewer/kinematic_teach_state.h"
#include "kinematic_viewer/kinematic_viewer_config.h"
#include "kinematic_viewer/rkv_panel_plugin.h"
#include "kinematic_viewer/rkv_panel_registry.h"
#include "rkv/ik_solver.h"
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
#include <vector>

using kinematic_viewer::appendCircle;
using kinematic_viewer::appendMarkerAxes;
using kinematic_viewer::CollisionMonitor;
using kinematic_viewer::CollisionMonitorResult;
using kinematic_viewer::CollisionMonitorState;
using kinematic_viewer::createKinematicLineProgram;
using kinematic_viewer::createKinematicMeshProgram;
using kinematic_viewer::createKinematicPointProgram;
using kinematic_viewer::DebugPlaybackState;
using kinematic_viewer::DestroyUserObstacleGpuMeshes;
using kinematic_viewer::distancePointToSegment2D;
using kinematic_viewer::DrawUserObstacles;
using kinematic_viewer::FocusCameraOnLink;
using kinematic_viewer::GetScrollDelta;
using kinematic_viewer::IkState;
using kinematic_viewer::InitialPoseApplyResult;
using kinematic_viewer::InitUserObstacleGpuMeshes;
using kinematic_viewer::KinematicApp;
using kinematic_viewer::KinematicIkController;
using kinematic_viewer::KinematicInputHandler;
using kinematic_viewer::KinematicLineRenderer;
using kinematic_viewer::KinematicLineVertex;
using kinematic_viewer::KinematicPointCloudLayer;
using kinematic_viewer::KinematicRenderLoop;
using kinematic_viewer::KinematicUiFeedback;
using kinematic_viewer::KinematicViewerConfig;
using kinematic_viewer::LaunchConfig;
using kinematic_viewer::LinkKinematicsAnalyzer;
using kinematic_viewer::LoadLaunchConfigFromArgs;
using kinematic_viewer::LoadTeachProgramFromYaml;
using kinematic_viewer::markerWorldMatrix;
using kinematic_viewer::MergeUserObstaclesIntoCollisionResult;
using kinematic_viewer::PointCloudColorModeFromString;
using kinematic_viewer::PointCloudLoadOptions;
using kinematic_viewer::PointCloudUiState;
using kinematic_viewer::RenderJointPanel;
using kinematic_viewer::RenderLinkInspectorPanel;
using kinematic_viewer::RenderObstaclePanel;
using kinematic_viewer::RenderPlaybackPanel;
using kinematic_viewer::RenderPointCloudPanel;
using kinematic_viewer::RenderRobotTreePanel;
using kinematic_viewer::RenderSafetyPanel;
using kinematic_viewer::RenderScenePanel;
using kinematic_viewer::RenderTeachPanel;
using kinematic_viewer::RenderTfPanel;
using kinematic_viewer::TeachProgramState;
using kinematic_viewer::TrajectoryPlayer;
using kinematic_viewer::UiSemanticLevel;
using kinematic_viewer::UserObstacleGpuMeshes;
using kinematic_viewer::ViewerState;
using kinematic_viewer::worldToScreen;
using kinematic_viewer::wrapDeltaDeg;
using rkv::IkSolveStats;
using rkv::OrbitCamera;
using rkv::RobotScene;

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

}  // namespace robot_kinematic_viewer_internal

using robot_kinematic_viewer_internal::ComputeWorldRayFromScreen;
using robot_kinematic_viewer_internal::IntersectRaySphere;
using robot_kinematic_viewer_internal::IsMobileBaseDragRobot;
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

    GLuint mesh_shader  = createKinematicMeshProgram();
    GLuint line_shader  = createKinematicLineProgram();
    GLuint point_shader = createKinematicPointProgram();
    KinematicLineRenderer line_renderer;
    line_renderer.init();

    KinematicRenderLoop render_loop;
    render_loop.line_renderer = &line_renderer;

    kinematic_viewer::VideoRecorder video_recorder;

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
    ui_state.lock_base                  = cfg.ui.fix_base_like_mujoco;
    ui_state.mobile_base_drag_available = cfg.ui.enable_mobile_base_drag && IsMobileBaseDragRobot(urdf_path, cfg.ui.mobile_base_robots);
    ui_state.mobile_base_drag_enabled   = ui_state.mobile_base_drag_available;
    auto appendJointInputGroup          = [&](const std::string& name, const std::vector<std::string>& joint_names) {
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

    PointCloudUiState point_cloud_state;
    KinematicPointCloudLayer point_cloud_layer;
    point_cloud_state.visible       = cfg.point_cloud.visible;
    point_cloud_state.voxel_size    = cfg.point_cloud.voxel_size;
    point_cloud_state.max_points    = cfg.point_cloud.max_points;
    point_cloud_state.x_min         = cfg.point_cloud.x_min;
    point_cloud_state.x_max         = cfg.point_cloud.x_max;
    point_cloud_state.y_min         = cfg.point_cloud.y_min;
    point_cloud_state.y_max         = cfg.point_cloud.y_max;
    point_cloud_state.z_min         = cfg.point_cloud.z_min;
    point_cloud_state.z_max         = cfg.point_cloud.z_max;
    point_cloud_state.offset_x      = cfg.point_cloud.offset_x;
    point_cloud_state.offset_y      = cfg.point_cloud.offset_y;
    point_cloud_state.offset_yaw    = cfg.point_cloud.offset_yaw;
    point_cloud_state.point_size_px = cfg.point_cloud.point_size_px;
    if (cfg.point_cloud.color_mode == "intensity") {
        point_cloud_state.color_mode = 2;
    } else if (cfg.point_cloud.color_mode == "flat") {
        point_cloud_state.color_mode = 0;
    } else {
        point_cloud_state.color_mode = 1;
    }
    point_cloud_state.build_esdf           = cfg.point_cloud.build_esdf;
    point_cloud_state.esdf_resolution      = cfg.point_cloud.esdf_resolution;
    point_cloud_state.esdf_map_origin[0]   = cfg.point_cloud.esdf_map_origin[0];
    point_cloud_state.esdf_map_origin[1]   = cfg.point_cloud.esdf_map_origin[1];
    point_cloud_state.esdf_map_origin[2]   = cfg.point_cloud.esdf_map_origin[2];
    point_cloud_state.esdf_map_size[0]     = cfg.point_cloud.esdf_map_size[0];
    point_cloud_state.esdf_map_size[1]     = cfg.point_cloud.esdf_map_size[1];
    point_cloud_state.esdf_map_size[2]     = cfg.point_cloud.esdf_map_size[2];
    point_cloud_state.esdf_use_fixed_map   = cfg.point_cloud.esdf_use_fixed_map;
    point_cloud_state.esdf_max_visual_dist = cfg.point_cloud.esdf_max_visual_dist;
    point_cloud_state.esdf_visual_stride   = cfg.point_cloud.esdf_visual_stride;
    point_cloud_state.esdf_visual_mode     = kinematic_viewer::EsdfVisualModeFromString(cfg.point_cloud.esdf_visual_mode);
    point_cloud_state.esdf_color_mode      = kinematic_viewer::EsdfColorModeFromString(cfg.point_cloud.esdf_color_mode);
    point_cloud_state.esdf_z_slice_enable  = cfg.point_cloud.esdf_z_slice_enable;
    point_cloud_state.esdf_z_slice_m       = cfg.point_cloud.esdf_z_slice_m;
    point_cloud_state.esdf_use_raycast     = cfg.point_cloud.esdf_use_raycast;
    point_cloud_state.esdf_ray_origin[0]   = cfg.point_cloud.esdf_ray_origin[0];
    point_cloud_state.esdf_ray_origin[1]   = cfg.point_cloud.esdf_ray_origin[1];
    point_cloud_state.esdf_ray_origin[2]   = cfg.point_cloud.esdf_ray_origin[2];
    point_cloud_state.esdf_ray_origin_auto = cfg.point_cloud.esdf_ray_origin_auto;
    point_cloud_state.esdf_min_ray_length  = cfg.point_cloud.esdf_min_ray_length;
    point_cloud_state.esdf_max_ray_length  = cfg.point_cloud.esdf_max_ray_length;
    if (!cfg.point_cloud.file_path.empty()) {
        std::snprintf(point_cloud_state.file_path, sizeof(point_cloud_state.file_path), "%s", cfg.point_cloud.file_path.c_str());
    }
    if (cfg.point_cloud.enable && cfg.point_cloud.auto_load_on_start && !cfg.point_cloud.file_path.empty()) {
        std::string pcd_path = cfg.point_cloud.file_path;
        if (!std::filesystem::path(pcd_path).is_absolute()) {
            std::filesystem::path resolved;
            if (!launch.configPath.empty()) {
                resolved = std::filesystem::path(launch.configPath).parent_path().parent_path() / pcd_path;
            }
            if (!resolved.empty() && std::filesystem::exists(resolved)) {
                pcd_path = resolved.lexically_normal().string();
            } else if (std::filesystem::exists(pcd_path)) {
                pcd_path = std::filesystem::absolute(pcd_path).lexically_normal().string();
            }
        }
        PointCloudLoadOptions load_opt;
        load_opt.voxel_size           = point_cloud_state.voxel_size;
        load_opt.max_points           = point_cloud_state.max_points;
        load_opt.x_min                = point_cloud_state.x_min;
        load_opt.x_max                = point_cloud_state.x_max;
        load_opt.y_min                = point_cloud_state.y_min;
        load_opt.y_max                = point_cloud_state.y_max;
        load_opt.z_min                = point_cloud_state.z_min;
        load_opt.z_max                = point_cloud_state.z_max;
        load_opt.color_mode           = PointCloudColorModeFromString(cfg.point_cloud.color_mode);
        load_opt.build_esdf           = cfg.point_cloud.build_esdf;
        load_opt.esdf_resolution      = cfg.point_cloud.esdf_resolution;
        load_opt.esdf_map_origin[0]   = cfg.point_cloud.esdf_map_origin[0];
        load_opt.esdf_map_origin[1]   = cfg.point_cloud.esdf_map_origin[1];
        load_opt.esdf_map_origin[2]   = cfg.point_cloud.esdf_map_origin[2];
        load_opt.esdf_map_size[0]     = cfg.point_cloud.esdf_map_size[0];
        load_opt.esdf_map_size[1]     = cfg.point_cloud.esdf_map_size[1];
        load_opt.esdf_map_size[2]     = cfg.point_cloud.esdf_map_size[2];
        load_opt.esdf_use_fixed_map   = cfg.point_cloud.esdf_use_fixed_map;
        load_opt.esdf_max_visual_dist = cfg.point_cloud.esdf_max_visual_dist;
        load_opt.esdf_visual_stride   = cfg.point_cloud.esdf_visual_stride;
        load_opt.esdf_visual_mode     = kinematic_viewer::EsdfVisualModeFromString(cfg.point_cloud.esdf_visual_mode);
        load_opt.esdf_color_mode      = kinematic_viewer::EsdfColorModeFromString(cfg.point_cloud.esdf_color_mode);
        load_opt.esdf_z_slice_enable  = cfg.point_cloud.esdf_z_slice_enable;
        load_opt.esdf_z_slice_m       = cfg.point_cloud.esdf_z_slice_m;
        load_opt.esdf_use_raycast     = cfg.point_cloud.esdf_use_raycast;
        load_opt.esdf_ray_origin[0]   = cfg.point_cloud.esdf_ray_origin[0];
        load_opt.esdf_ray_origin[1]   = cfg.point_cloud.esdf_ray_origin[1];
        load_opt.esdf_ray_origin[2]   = cfg.point_cloud.esdf_ray_origin[2];
        load_opt.esdf_ray_origin_auto = cfg.point_cloud.esdf_ray_origin_auto;
        load_opt.esdf_min_ray_length  = cfg.point_cloud.esdf_min_ray_length;
        load_opt.esdf_max_ray_length  = cfg.point_cloud.esdf_max_ray_length;
        if (cfg.point_cloud.build_esdf && cfg.point_cloud.point_size_px >= 2.0f) {
            point_cloud_state.point_size_px = cfg.point_cloud.point_size_px;
        }
        std::string status;
        if (point_cloud_layer.LoadFromFile(pcd_path, load_opt, &status)) {
            point_cloud_state.loaded          = true;
            point_cloud_state.rendered_points = point_cloud_layer.gpuPointCount();
            std::snprintf(point_cloud_state.file_path, sizeof(point_cloud_state.file_path), "%s", pcd_path.c_str());
        }
        std::snprintf(point_cloud_state.last_status, sizeof(point_cloud_state.last_status), "%s", status.c_str());
    }

    // ---------------------------------------------------------------------------
    // Panel plugin registry — loads panel .so files from lib/ by id or path.
    // ---------------------------------------------------------------------------
    kinematic_viewer::RkvPanelRegistry panel_registry;
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

        std::vector<std::string> specs = cfg.ui.sidebar_panels;
        if (specs.empty()) {
            specs = {"scene", "ik", "playback", "safety", "joint", "tf", "obstacle", "planner", "teach", "point_cloud"};
        }
        panel_registry.LoadFromConfig(specs, panel_search_dirs);
        if (panel_registry.Count() < static_cast<int>(specs.size())) {
            std::cerr << "[robot_kinematic_viewer] sidebar panels: loaded " << panel_registry.Count() << "/"
                      << specs.size() << ". Missing plugins are usually stale librkv_panel_*.so — run a full build.\n";
        }
    }
    {
        const int preferred = panel_registry.IndexOf("joint");
        ui_state.sidebar_page = preferred >= 0 ? preferred : 0;
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
    {
        auto isExistingDir = [](const std::string& path) -> bool {
            std::error_code ec;
            return std::filesystem::exists(path, ec) && std::filesystem::is_directory(path, ec);
        };
        std::string initial_browser_dir;
        if (!cfg.playback.last_browser_dir.empty() && isExistingDir(cfg.playback.last_browser_dir)) {
            initial_browser_dir = kinematic_viewer::NormalizePath(cfg.playback.last_browser_dir);
        } else {
            const std::string trajectories_dir = kinematic_viewer::NormalizePath("config/trajectories");
            if (isExistingDir(trajectories_dir)) {
                initial_browser_dir = trajectories_dir;
            } else if (!playback_state.trajectory_files.empty()) {
                initial_browser_dir = kinematic_viewer::NormalizePath(
                    std::filesystem::path(playback_state.trajectory_files.front().path).parent_path().string());
            } else {
                const char* home    = std::getenv("HOME");
                initial_browser_dir = (home != nullptr && home[0] != '\0')
                                          ? kinematic_viewer::NormalizePath(home)
                                          : kinematic_viewer::NormalizePath(std::filesystem::current_path().string());
            }
        }
        std::snprintf(playback_state.trajectory_browser_dir, sizeof(playback_state.trajectory_browser_dir), "%s",
                      initial_browser_dir.c_str());
    }
    kinematic_viewer::PlaybackStateMachine playback_sm(&playback_state);
    TeachProgramState teach_state;
    for (const auto& path : cfg.teach.program_files) {
        kinematic_viewer::TeachFileEntry entry;
        entry.path   = path;
        entry.status = "未加载";
        entry.loaded = false;
        teach_state.program_files.push_back(std::move(entry));
    }
    if (!teach_state.program_files.empty()) {
        teach_state.selected_program_index =
            std::clamp(cfg.teach.selected_index, 0, static_cast<int>(teach_state.program_files.size()) - 1);
        std::snprintf(teach_state.program_file_path, sizeof(teach_state.program_file_path), "%s",
                      teach_state.program_files[teach_state.selected_program_index].path.c_str());
    }
    {
        auto isExistingDir = [](const std::string& path) -> bool {
            std::error_code ec;
            return std::filesystem::exists(path, ec) && std::filesystem::is_directory(path, ec);
        };
        std::string teach_browser_dir;
        if (!cfg.teach.last_browser_dir.empty() && isExistingDir(cfg.teach.last_browser_dir)) {
            teach_browser_dir = kinematic_viewer::NormalizePath(cfg.teach.last_browser_dir);
        } else {
            const std::string teach_dir = kinematic_viewer::NormalizePath("config/teach");
            teach_browser_dir           = isExistingDir(teach_dir) ? teach_dir : playback_state.trajectory_browser_dir;
        }
        std::snprintf(teach_state.program_browser_dir, sizeof(teach_state.program_browser_dir), "%s", teach_browser_dir.c_str());
    }
    if (teach_state.selected_program_index >= 0) {
        std::string teach_load_error;
        auto& selected_entry = teach_state.program_files[static_cast<size_t>(teach_state.selected_program_index)];
        if (LoadTeachProgramFromYaml(teach_state.program_file_path, &teach_state, &teach_load_error)) {
            selected_entry.status = "加载成功";
            selected_entry.loaded = true;
            teach_state.io_status = "启动已加载: " + teach_state.program_name;
        } else {
            selected_entry.status = "加载失败: " + teach_load_error;
            selected_entry.loaded = false;
            teach_state.io_status = selected_entry.status;
        }
    }
    CollisionMonitorState collision_state;
    CollisionMonitorResult collision_result;
    TrajectoryPlayer trajectory_player;
    CollisionMonitor collision_monitor;
    LinkKinematicsAnalyzer link_kinematics_analyzer;
    KinematicUiFeedback ui_feedback;
    kinematic_viewer::KinematicIkController ik_controller(&ik_state);
    std::string last_playback_io_status;
    bool initial_pose_auto_apply_pending = cfg.initial_pose.enable && cfg.initial_pose.auto_apply_on_start;
    int collision_refresh_tick             = 0;
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
        if (!ik_state.chains.empty()) {
            ik_state.selected_chain = std::clamp(ik_state.selected_chain, 0, static_cast<int>(ik_state.chains.size()) - 1);
        }
        ik_state.marker_targets.resize(ik_state.chains.size());
    }
    ik_state.use_external_target = false;

    double prev_x                 = 0.0;
    double prev_y                 = 0.0;
    bool first_mouse              = true;
    bool obstacle_pick_left_prev  = false;
    bool obstacle_gizmo_was_using = false;
    bool obstacle_gizmo_was_over  = false;
    bool base_gizmo_was_using     = false;
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
        const int panel_w    = ui_state.sidebar_hidden ? 0 : static_cast<int>(ui_state.panel_width);
        int viewport_w       = std::max(1, fb_w - panel_w);
        int viewport_h       = std::max(1, fb_h);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        ImGuizmo::BeginFrame();

        const bool sidebar_hotkeys_enabled = !ImGui::GetIO().WantTextInput && !ImGui::GetIO().WantCaptureKeyboard;
        ui_state.sidebar_page =
            input_handler.HandleSidebarHotkeys(ui_state.sidebar_page, panel_registry.Count(), sidebar_hotkeys_enabled);

        const auto viewport_hotkeys =
            input_handler.HandleViewportHotkeys(sidebar_hotkeys_enabled, playback_sm.HasKeyframes());
        if (viewport_hotkeys.toggled_sidebar) {
            ui_state.sidebar_hidden = !ui_state.sidebar_hidden;
            ui_feedback.Push(UiSemanticLevel::Info, ui_state.sidebar_hidden ? "已隐藏侧栏 (H 恢复)" : "已显示侧栏", now_sec);
        }
        if (viewport_hotkeys.toggled_playback) {
            if (playback_sm.TogglePlayPause()) {
                ui_feedback.Push(UiSemanticLevel::Info, playback_sm.IsPlaying() ? "回放: 播放" : "回放: 暂停", now_sec, 1.2);
            }
        }
        if (panel_registry.Count() == 0) {
            ui_state.sidebar_page = 0;
        } else {
            ui_state.sidebar_page = std::clamp(ui_state.sidebar_page, 0, panel_registry.Count() - 1);
        }
        const std::string current_panel_key =
            panel_registry.Count() == 0 ? std::string("scene") : panel_registry.Id(ui_state.sidebar_page);
        ui_state.scene_panel_active = (current_panel_key == "scene");

        KinematicInputHandler::UpdateContext input_ctx;
        input_ctx.mouse_x             = x;
        input_ctx.mouse_y             = y;
        input_ctx.viewport_w          = viewport_w;
        input_ctx.viewport_h          = viewport_h;
        input_ctx.imgui_wants_mouse   = ImGui::GetIO().WantCaptureMouse;
        input_ctx.panel_resize_active = ui_state.panel_resize_active;
        input_ctx.ik_gizmo_using      = ik_state.gizmo_was_using;
        input_ctx.ik_gizmo_over       = ik_state.gizmo_was_over;
        input_ctx.obs_gizmo_using     = obstacle_gizmo_was_using || base_gizmo_was_using;
        input_ctx.obs_gizmo_over      = obstacle_gizmo_was_over;
        input_ctx.ik_dragging_marker  = ik_state.dragging_marker;
        input_ctx.sidebar_page        = (current_panel_key == "scene") ? 0 : ((current_panel_key == "obstacle") ? 6 : -1);
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

        const auto run_collision_monitor = [&]() {
            if (!collision_state.enable) {
                return;
            }
            collision_result = collision_monitor.Evaluate(collision_state, scene);
            MergeUserObstaclesIntoCollisionResult(ui_state.user_obstacles, scene, collision_state.warning_distance_m,
                                                  collision_state.danger_distance_m, &collision_result);
            collision_monitor.UpdateStateFromResult(collision_result, &collision_state);
        };

        if (collision_state.enable &&
            (scene.isJointPoseDirty() || playback_sm.IsPlaying() || (++collision_refresh_tick % 4 == 0))) {
            run_collision_monitor();
        }

        glm::mat4 proj =
            glm::perspective(glm::radians(50.0f), static_cast<float>(viewport_w) / static_cast<float>(viewport_h), 0.05f, 80.0f);
        glm::mat4 view = camera.viewMatrix();

        KinematicRenderLoop::Context render_ctx;
        render_ctx.viewport_w        = viewport_w;
        render_ctx.viewport_h        = viewport_h;
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

        kinematic_viewer::CaptureFrameForRecorder(&video_recorder, viewport_w, viewport_h);

        auto applyIkForActiveChain = [&](bool force_orientation_lock, bool fast_mode, bool prefer_position_only_target) -> bool {
            return ik_controller.ApplyIkForActiveChain(&scene, force_orientation_lock, fast_mode, prefer_position_only_target);
        };

        auto refineActiveChainToMarker = [&]() -> bool {
            return ik_controller.RefineActiveChainToMarker(&scene);
        };

        auto activeChainPositionErrorMmToMarker = [&]() -> float {
            return ik_controller.ActiveChainPositionErrorMmToMarker(&scene);
        };

        // External target path disabled; keep local marker update path.
        ik_controller.ApplyExternalTarget(&scene);

        // RViz-like manipulator via ImGuizmo
        // Draw gizmo directly on the foreground drawlist of the 3D viewport area.
        ImGuizmo::SetOrthographic(false);
        ImGuizmo::AllowAxisFlip(false);
        ImGuizmo::SetDrawlist(ImGui::GetForegroundDrawList());
        ImGuizmo::SetRect(0.0f, 0.0f, static_cast<float>(viewport_w), static_cast<float>(viewport_h));

        const bool obstacle_page_active = (current_panel_key == "scene" || current_panel_key == "obstacle");
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

        const bool base_edit_active =
            ui_state.mobile_base_drag_available && ui_state.mobile_base_drag_enabled && current_panel_key == "scene";
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
        const bool ik_page_active = (current_panel_key == "ik");
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

        if (ui_state.enable_link_hover_highlight) {
            auto link_hover = input_handler.UpdateLinkHover(input_ctx, pick_view, pick_proj, &scene, glfwGetTime());
            if (!link_hover.throttle_skip) {
                ui_state.hovered_link = link_hover.picked ? link_hover.link_name : std::string();
            }
        } else {
            ui_state.hovered_link.clear();
        }

        if (ui_state.enable_link_click_select) {
            auto link_pick = input_handler.UpdateLinkPick(input_ctx, pick_view, pick_proj, &scene);
            if (link_pick.picked) {
                ui_state.selected_link            = link_pick.link_name;
                ui_state.trajectory_min_surface_m = -1.0f;
                ui_state.selected_joint           = -1;
            }
        }

        input_ctx.ik_gizmo_using     = ik_state.gizmo_was_using;
        input_ctx.ik_dragging_marker = ik_state.dragging_marker;
        input_ctx.obs_gizmo_using    = obstacle_gizmo_was_using || base_gizmo_was_using;
        input_ctx.imgui_wants_mouse  = ImGui::GetIO().WantCaptureMouse;
        input_handler.UpdateCamera(&camera, input_ctx);

        {
            kinematic_viewer::ViewportHudContext hud_ctx;
            hud_ctx.viewport_w        = viewport_w;
            hud_ctx.viewport_h        = viewport_h;
            hud_ctx.ui_state          = &ui_state;
            hud_ctx.playback_state    = &playback_state;
            hud_ctx.playback_sm       = &playback_sm;
            hud_ctx.playback_player   = &trajectory_player;
            hud_ctx.scene             = &scene;
            hud_ctx.collision_state   = &collision_state;
            hud_ctx.collision_result  = &collision_result;
            hud_ctx.video_recorder    = &video_recorder;
            kinematic_viewer::RenderViewportHud(hud_ctx);
        }

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

        if (ImGui::CollapsingHeader("视频录制")) {
            kinematic_viewer::RenderVideoRecorderPanel(&ui_state, &video_recorder, &ui_feedback, now_sec, viewport_w, viewport_h);
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

            ImGui::BeginChild("##top_status_chips", ImVec2(0.0f, 34.0f), false,
                              ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
            KinematicUiFeedback::RenderStatusChip(ik_state.solve_mode == "full_body" ? "IK FULL_BODY" : "IK SINGLE", UiSemanticLevel::Info);
            ImGui::SameLine();
            KinematicUiFeedback::RenderStatusChip(playbackLabel, playbackLevel);
            ImGui::SameLine();
            KinematicUiFeedback::RenderStatusChip(collisionLabel.c_str(), collisionLevel);
            ImGui::EndChild();
        }
        if (ImGui::CollapsingHeader("操作提示")) {
            ImGui::TextDisabled("视角：左键旋转，中键/Shift+左键平移，右键缩放，滚轮缩放");
            ImGui::TextDisabled("Space 播放/暂停  ·  H 隐藏/显示侧栏  ·  1-9 切换子页");
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
        if (current_panel_key == "scene" || current_panel_key == "tf") {
            RenderLinkInspectorPanel(&ui_state, &scene, &camera, &collision_state, &collision_result, &playback_state, &collision_monitor,
                                     &link_kinematics_analyzer);
        }

        auto joints   = scene.getJointInfos();
        auto tf_infos = scene.getLinkTfInfos();
        kinematic_viewer::TickTrajectorySequence(&playback_state, &playback_sm, was_playback_playing, joints, &trajectory_player, &scene);

        // Build the context bag and dispatch to the active panel plugin.
        RkvPanelCtx panel_ctx{};
        panel_ctx.viewer_state            = &ui_state;
        panel_ctx.ik_state                = &ik_state;
        panel_ctx.ik_controller           = &ik_controller;
        panel_ctx.scene                   = &scene;
        panel_ctx.camera                  = &camera;
        panel_ctx.collision_state         = &collision_state;
        panel_ctx.collision_result        = &collision_result;
        panel_ctx.collision_monitor       = &collision_monitor;
        panel_ctx.playback_state          = &playback_state;
        panel_ctx.playback_player         = &trajectory_player;
        panel_ctx.playback_sm             = &playback_sm;
        panel_ctx.teach_state             = &teach_state;
        panel_ctx.point_cloud_state       = &point_cloud_state;
        panel_ctx.point_cloud_layer       = &point_cloud_layer;
        panel_ctx.path_planner_ui         = &path_planner_ui;
        panel_ctx.link_kinematics_analyzer = &link_kinematics_analyzer;
        panel_ctx.joints                  = &joints;
        panel_ctx.tf_infos                = &tf_infos;
        panel_ctx.ik_solver               = &ik_state.solver;
        panel_ctx.ik_chains               = &ik_state.chains;

        panel_registry.Render(ui_state.sidebar_page, &panel_ctx);

        kinematic_viewer::EndSidebarScrollRegion();

        if (scene.consumeJointPoseDirty()) {
            scene.updateTransforms();
            run_collision_monitor();
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
                ui_feedback.Push(level, playback_state.trajectory_io_status, now_sec, level == UiSemanticLevel::Error ? 4.5 : 2.8);
            }
            last_playback_io_status = playback_state.trajectory_io_status;
        }

        ImGui::End();
        }

        ui_feedback.RenderToasts(now_sec, static_cast<float>(viewport_w) - 14.0f, 14.0f);

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        app.SwapBuffers();
    }

    // Persist trajectory file list to config before exit
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

    DestroyUserObstacleGpuMeshes(&obstacle_meshes);
    glDeleteProgram(mesh_shader);
    glDeleteProgram(line_shader);
    return 0;
}
