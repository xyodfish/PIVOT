#pragma once

#include "teleop_viewer/scene.h"
#include "teleop_viewer/ik_solver.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <memory>
#include <string>
#include <vector>

namespace kinematic_viewer {

    // A single waypoint in Cartesian space: [time, x, y, z, qx, qy, qz, qw]
    struct CartesianWaypoint {
        float time_sec = 0.0f;
        glm::vec3 position{0.0f};
        glm::quat orientation{1.0f, 0.0f, 0.0f, 0.0f};
    };

    // Result of planning a Cartesian path
    struct CartesianPathResult {
        std::vector<CartesianWaypoint> waypoints;
        std::string status;  // "成功" or error message
        bool success = false;
    };

    // Parameters for circle path
    struct CirclePathParams {
        glm::vec3 center{0.0f};              // Circle center in world frame
        glm::vec3 normal{0.0f, 0.0f, 1.0f};  // Plane normal
        float radius          = 0.1f;        // Radius in meters
        float period_sec      = 4.0f;        // Time for one full circle
        int num_points        = 60;          // Number of waypoints
        bool lock_orientation = true;        // Keep initial orientation
    };

    // Parameters for square path
    struct SquarePathParams {
        glm::vec3 center{0.0f};              // Square center in world frame
        glm::vec3 normal{0.0f, 0.0f, 1.0f};  // Plane normal
        float side_length     = 0.15f;       // Side length in meters
        float corner_radius   = 0.02f;       // Corner fillet radius (0 for sharp)
        float period_sec      = 4.0f;        // Time for one full square
        int num_points        = 80;          // Number of waypoints
        bool lock_orientation = true;
    };

    // Parameters for head bob (pitch oscillation)
    struct HeadBobParams {
        float pitch_amplitude_deg = 15.0f;  // Peak pitch angle
        float period_sec          = 2.0f;   // Time for one up-down cycle
        int num_points            = 40;
    };

    // Parameters for straight line path
    struct StraightPathParams {
        glm::vec3 start_pos{0.0f};
        glm::vec3 goal_pos{0.0f};
        glm::quat start_quat{1.0f, 0.0f, 0.0f, 0.0f};
        glm::quat goal_quat{1.0f, 0.0f, 0.0f, 0.0f};
        float max_vel       = 0.2f;    // m/s
        float max_acc       = 0.1f;    // m/s^2
        float delta_t       = 0.02f;   // Time step
        std::string profile = "DSVP";  // "TVP" or "DSVP"
    };

    // Parameters for joint-space point-to-point (PTP) velocity planning
    struct JointSpacePTPParams {
        std::vector<float> start_positions;    // Start joint positions (rad or m)
        std::vector<float> goal_positions;     // Goal joint positions (rad or m)
        std::vector<std::string> joint_names;  // Joint names
        float max_vel       = 1.0f;            // Max joint velocity (rad/s or m/s)
        float max_acc       = 2.0f;            // Max joint acceleration (rad/s^2 or m/s^2)
        float max_jerk      = 10.0f;           // Max joint jerk (rad/s^3 or m/s^3), used by DSVP
        float delta_t       = 0.02f;           // Time step for sampling
        std::string profile = "DSVP";          // "TVP" or "DSVP"
    };

    // Unified path planner interface
    class CartesianPathPlanner {
       public:
        virtual ~CartesianPathPlanner() = default;

        // Plan a path given the current tip pose as starting point
        virtual CartesianPathResult plan(const glm::vec3& current_pos, const glm::quat& current_quat) = 0;

        // Get planner name
        virtual std::string name() const = 0;
    };

    // Concrete planners
    std::unique_ptr<CartesianPathPlanner> makeCirclePlanner(const CirclePathParams& params);
    std::unique_ptr<CartesianPathPlanner> makeSquarePlanner(const SquarePathParams& params);
    std::unique_ptr<CartesianPathPlanner> makeHeadBobPlanner(const HeadBobParams& params);
    std::unique_ptr<CartesianPathPlanner> makeStraightPlanner(const StraightPathParams& params);

    // Convert Cartesian path to joint-space trajectory via IK
    // Returns a vector of joint state maps: [{joint_name -> position}, ...]
    struct JointSpaceTrajectory {
        std::vector<float> times;                         // Time for each waypoint
        std::vector<std::vector<float>> joint_positions;  // Per-joint positions per waypoint
        std::vector<std::string> joint_names;
        std::string status;
        bool success = false;
    };

    // Progress callback for IK solving: (current_index, total_count, status_message)
    using IkSolveProgressCallback = std::function<void(int, int, const std::string&)>;

    // IK solver wrapper for path planning (single chain mode)
    // Takes a Cartesian path and solves IK for each waypoint using single_chain mode
    // Returns joint names and positions for each waypoint
    JointSpaceTrajectory solveIkForCartesianPath(const CartesianPathResult& cartesian_path, teleop_viewer::RobotScene* scene,
                                                 teleop_viewer::IkSolver* solver, int chain_index,
                                                 const IkSolveProgressCallback& progress_cb = nullptr);

    // IK solver wrapper for path planning (full body mode)
    // Uses solveFullBody for better multi-chain coordination and solve quality
    // Other chains keep their current pose while the active chain follows the path
    JointSpaceTrajectory solveIkForCartesianPathFullBody(const CartesianPathResult& cartesian_path, teleop_viewer::RobotScene* scene,
                                                         teleop_viewer::IkSolver* solver, int chain_index,
                                                         const IkSolveProgressCallback& progress_cb = nullptr);

    // Joint-space point-to-point velocity planning using vp::MultiVelocityPlanner
    // Plans a time-synchronized trajectory for all joints from start to goal positions
    JointSpaceTrajectory planJointSpacePTP(const JointSpacePTPParams& params);

    // Export joint-space trajectory to CSV format compatible with playback
    bool exportTrajectoryToCsv(const JointSpaceTrajectory& traj, const std::string& file_path);

}  // namespace kinematic_viewer
