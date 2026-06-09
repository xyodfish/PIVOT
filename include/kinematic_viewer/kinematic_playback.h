#pragma once

#include "kinematic_viewer/kinematic_playback_state_machine.h"
#include "kinematic_viewer/kinematic_runtime_state.h"
#include "rkv/scene.h"

#include <memory>
#include <string>

namespace kinematic_viewer {

    class TrajectoryInterpolator {
       public:
        virtual ~TrajectoryInterpolator()                                   = default;
        virtual void SampleAndApply(const DebugPlaybackState& playbackState, float sampleTimeSec, int* currentSegmentIndex,
                                    rkv::RobotScene* scene) const = 0;
    };

    class LinearTrajectoryInterpolator : public TrajectoryInterpolator {
       public:
        void SampleAndApply(const DebugPlaybackState& playbackState, float sampleTimeSec, int* currentSegmentIndex,
                            rkv::RobotScene* scene) const override;
    };

    class TrajectoryPlayer {
       public:
        TrajectoryPlayer();

        void SetInterpolator(std::unique_ptr<TrajectoryInterpolator> interpolator);

        void RecordKeyframe(DebugPlaybackState* playbackState, const std::vector<rkv::RobotScene::JointInfo>& joints,
                            const rkv::RobotScene& scene) const;
        void RemoveSelectedKeyframe(DebugPlaybackState* playbackState) const;
        void Clear(DebugPlaybackState* playbackState) const;
        void TogglePlayPause(DebugPlaybackState* playbackState) const;
        void Stop(DebugPlaybackState* playbackState) const;
        void AdvanceAndApply(DebugPlaybackState* playbackState, rkv::RobotScene* scene, double dtSec) const;
        void SampleAtCurrentTime(const DebugPlaybackState& playbackState, rkv::RobotScene* scene) const;

        static float TotalDuration(const DebugPlaybackState& playbackState);
        static bool HasPlayableTrajectory(const DebugPlaybackState& playbackState);

       private:
        std::unique_ptr<TrajectoryInterpolator> interpolator_;
    };

    bool LoadTrajectoryFromFile(const std::string& path, DebugPlaybackState* playbackState, std::string* errorMessage);
    bool SaveTrajectoryToFile(const std::string& path, const DebugPlaybackState& playbackState, std::string* errorMessage);
    void BuildDemoTrajectoryFromCurrentPose(DebugPlaybackState* playbackState,
                                            const std::vector<rkv::RobotScene::JointInfo>& joints,
                                            const rkv::RobotScene& scene);

    bool ValidateTrajectoryJointNames(const DebugPlaybackState& playbackState,
                                      const std::vector<rkv::RobotScene::JointInfo>& joints, std::string* errorMessage);
    bool LoadTrajectoryListEntry(DebugPlaybackState* playbackState, int index,
                                 const std::vector<rkv::RobotScene::JointInfo>& joints, TrajectoryPlayer* playbackPlayer,
                                 rkv::RobotScene* scene);
    void ProcessPendingTrajectoryLoad(DebugPlaybackState* playbackState, const std::vector<rkv::RobotScene::JointInfo>& joints,
                                      TrajectoryPlayer* playbackPlayer, rkv::RobotScene* scene,
                                      PlaybackStateMachine* playback_sm);
    void StartTrajectorySequence(DebugPlaybackState* playbackState, const std::vector<rkv::RobotScene::JointInfo>& joints,
                                 TrajectoryPlayer* playbackPlayer, rkv::RobotScene* scene, PlaybackStateMachine* playback_sm);
    void TickTrajectorySequence(DebugPlaybackState* playbackState, PlaybackStateMachine* playback_sm, bool was_playing_last_frame,
                                const std::vector<rkv::RobotScene::JointInfo>& joints, TrajectoryPlayer* playbackPlayer,
                                rkv::RobotScene* scene);
    void CancelTrajectorySequence(DebugPlaybackState* playbackState);

}  // namespace kinematic_viewer
