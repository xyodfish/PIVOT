#pragma once

#include "kinematic_viewer/kinematic_collision_monitor.h"
#include "kinematic_viewer/kinematic_line_renderer.h"
#include "kinematic_viewer/kinematic_marker_target_state.h"
#include "kinematic_viewer/kinematic_marker_utils.h"
#include "kinematic_viewer/kinematic_path_planner.h"
#include "kinematic_viewer/kinematic_runtime_state.h"
#include "kinematic_viewer/kinematic_point_cloud.h"
#include "kinematic_viewer/kinematic_user_obstacles.h"
#include "teleop_viewer/scene.h"

#include <glad/glad.h>
#include <glm/mat4x4.hpp>

#include <vector>

namespace kinematic_viewer {

    // Encapsulates all 3D viewport rendering logic: scene meshes, line overlays,
    // obstacles, grid/axes, and collision visualization.
    struct KinematicRenderLoop {
        // Per-frame render context (populated by the caller before Render()).
        struct Context {
            int viewport_w = 0;
            int viewport_h = 0;

            GLuint mesh_shader  = 0;
            GLuint line_shader  = 0;
            GLuint point_shader = 0;

            teleop_viewer::RobotScene* scene                  = nullptr;
            const PointCloudUiState* point_cloud              = nullptr;
            const KinematicPointCloudLayer* point_cloud_layer = nullptr;
            const ViewerState* ui_state                       = nullptr;
            const IkState* ik_state                           = nullptr;
            const CollisionMonitorState* collision_state      = nullptr;
            const CollisionMonitorResult* collision_result    = nullptr;
            const UserObstacleGpuMeshes* obstacle_meshes      = nullptr;

            const teleop_viewer::OrbitCamera* camera = nullptr;

            // Path planning preview (optional, may be null)
            const std::vector<CartesianWaypoint>* planned_path = nullptr;
            bool show_planned_path                             = false;
        };

        // Persistent GPU resources managed externally (caller owns lifecycle).
        KinematicLineRenderer* line_renderer = nullptr;

        // Render the full 3D viewport given the current context.
        // This sets up OpenGL state, draws the scene, obstacles, grid/axes, markers,
        // and collision visualization lines.
        void Render(const Context& ctx);

       private:
        void SetupViewportAndClear(int w, int h);
        void SetupMeshShaderUniforms(GLuint shader, const glm::mat4& proj, const glm::mat4& view, const glm::vec3& eye);
        void DrawSceneMeshes(GLuint shader, const Context& ctx, const glm::mat4& proj, const glm::mat4& view);
        void DrawObstacles(GLuint shader, const Context& ctx, const glm::mat4& view, const glm::mat4& proj);
        void DrawPointCloud(const Context& ctx, const glm::mat4& view, const glm::mat4& proj);
        void BuildAxisVertices(const Context& ctx, std::vector<KinematicLineVertex>* out);
        void BuildMarkerAxes(const Context& ctx, std::vector<KinematicLineVertex>* out);
        void BuildMobileBaseAxes(const Context& ctx, std::vector<KinematicLineVertex>* out);
        void BuildGridLines(const Context& ctx, std::vector<KinematicLineVertex>* out);
        void BuildWorldAxes(const Context& ctx, std::vector<KinematicLineVertex>* out);
        void BuildJointAxes(const Context& ctx, std::vector<KinematicLineVertex>* out);
        void BuildCollisionLine(const Context& ctx, std::vector<KinematicLineVertex>* out);
        void BuildPlannedPathLines(const Context& ctx, std::vector<KinematicLineVertex>* out);
        void BuildSelectedLinkHighlight(const Context& ctx, std::vector<KinematicLineVertex>* out);
        void DrawLineOverlays(GLuint shader, const Context& ctx, const std::vector<KinematicLineVertex>& vertices, const glm::mat4& view,
                              const glm::mat4& proj);
    };

}  // namespace kinematic_viewer
