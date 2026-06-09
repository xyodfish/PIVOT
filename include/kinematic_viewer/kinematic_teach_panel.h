#pragma once

#include "kinematic_viewer/kinematic_playback_state.h"
#include "kinematic_viewer/kinematic_teach_state.h"
#include "rkv/ik_solver.h"
#include "rkv/scene.h"

#include <vector>

namespace kinematic_viewer {

    void RenderTeachPanel(TeachProgramState* teach, DebugPlaybackState* playback_state, rkv::RobotScene* scene,
                          rkv::IkSolver* solver, const std::vector<rkv::IkChainStatus>& chains,
                          const std::vector<rkv::RobotScene::JointInfo>& joints);

}  // namespace kinematic_viewer
