#pragma once

#include "kinematic_viewer/kinematic_collision_monitor.h"
#include "kinematic_viewer/kinematic_playback_state.h"
#include "kinematic_viewer/kinematic_playback_state_machine.h"
#include "kinematic_viewer/kinematic_video_recorder.h"
#include "kinematic_viewer/kinematic_viewer_state.h"
#include "rkv/scene.h"

namespace kinematic_viewer {

    class TrajectoryPlayer;

    struct ViewportHudContext {
        int viewport_w = 0;
        int viewport_h = 0;

        const ViewerState* ui_state        = nullptr;
        DebugPlaybackState* playback_state = nullptr;
        PlaybackStateMachine* playback_sm  = nullptr;
        TrajectoryPlayer* playback_player  = nullptr;
        rkv::RobotScene* scene             = nullptr;

        const CollisionMonitorState* collision_state   = nullptr;
        const CollisionMonitorResult* collision_result = nullptr;

        const VideoRecorder* video_recorder = nullptr;
    };

    void RenderViewportHud(const ViewportHudContext& ctx);

}  // namespace kinematic_viewer
