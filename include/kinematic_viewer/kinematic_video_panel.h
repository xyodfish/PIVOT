#pragma once

#include "kinematic_viewer/kinematic_ui_feedback.h"

namespace kinematic_viewer {
    class VideoRecorder;
    struct ViewerState;

    void CaptureFrameForRecorder(VideoRecorder* video_recorder, int viewport_w, int viewport_h);
    void RenderVideoRecorderPanel(ViewerState* ui_state, VideoRecorder* video_recorder, KinematicUiFeedback* ui_feedback, double now_sec,
                                  int viewport_w, int viewport_h);
}  // namespace kinematic_viewer
