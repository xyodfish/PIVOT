#pragma once

#include "kinematic_viewer/kinematic_path_planner.h"
#include "kinematic_viewer/kinematic_playback_state.h"
#include "kinematic_viewer/kinematic_teach_state.h"
#include "teleop_viewer/ik_solver.h"
#include "teleop_viewer/scene.h"

#include <string>
#include <vector>

namespace kinematic_viewer {

    void CaptureTeachPointFromScene(TeachProgramState* teach, const std::vector<teleop_viewer::RobotScene::JointInfo>& joints,
                                     const teleop_viewer::RobotScene& scene, const std::string& ee_tip_link);

    void ApplyTeachPointToScene(const TeachPoint& point, teleop_viewer::RobotScene* scene);

    void RemoveSelectedTeachPoint(TeachProgramState* teach);
    void MoveTeachPoint(TeachProgramState* teach, int from_index, int to_index);
    void RenameTeachPoint(TeachProgramState* teach, int index, const std::string& new_name);

    bool SaveTeachProgramToYaml(const std::string& path, const TeachProgramState& teach, std::string* error_message);
    bool LoadTeachProgramFromYaml(const std::string& path, TeachProgramState* teach, std::string* error_message);

    /// 示教点序列 moveJ（关节空间 PTP，vp DoubleS/TVP）。
    JointSpaceTrajectory BuildTeachMoveJTrajectory(const TeachProgramState& teach, std::string* status_message);

    /// 示教点序列 moveL（笛卡尔直线段拼接；需示教点含末端位姿）。
    CartesianPathResult BuildTeachMoveLTrajectory(const TeachProgramState& teach, std::string* status_message);

    /// 将轨迹或示教点写入回放关键帧（供侧栏回放页播放）。
    bool LoadJointTrajectoryIntoPlayback(const JointSpaceTrajectory& traj, DebugPlaybackState* playback_state,
                                         std::string* error_message);
    bool LoadTeachPointsIntoPlayback(const TeachProgramState& teach, float seconds_per_segment,
                                     DebugPlaybackState* playback_state, std::string* error_message);

}  // namespace kinematic_viewer
