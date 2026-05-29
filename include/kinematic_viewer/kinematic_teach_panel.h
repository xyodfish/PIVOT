#pragma once

#include "kinematic_viewer/kinematic_playback_state.h"
#include "kinematic_viewer/kinematic_teach_state.h"
#include "teleop_viewer/ik_solver.h"
#include "teleop_viewer/scene.h"

#include <vector>

namespace kinematic_viewer {

    void RenderTeachPanel(TeachProgramState* teach, DebugPlaybackState* playback_state, teleop_viewer::RobotScene* scene,
                          teleop_viewer::IkSolver* solver, const std::vector<teleop_viewer::IkChainStatus>& chains,
                          const std::vector<teleop_viewer::RobotScene::JointInfo>& joints);

}  // namespace kinematic_viewer
