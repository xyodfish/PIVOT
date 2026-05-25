#pragma once

#include "kinematic_viewer/kinematic_runtime_state.h"
#include "teleop_viewer/scene.h"

#include <memory>
#include <string>

namespace kinematic_viewer {

    class TrajectoryInterpolator {
       public:
        virtual ~TrajectoryInterpolator()                                             = default;
        virtual void SampleAndApply(const DebugPlaybackState& playbackState, float sampleTimeSec, int* currentSegmentIndex,
                                    teleop_viewer::RobotScene* scene) const = 0;
    };

    class LinearTrajectoryInterpolator : public TrajectoryInterpolator {
       public:
        void SampleAndApply(const DebugPlaybackState& playbackState, float sampleTimeSec, int* currentSegmentIndex,
                            teleop_viewer::RobotScene* scene) const override;
    };

    class TrajectoryPlayer {
       public:
        TrajectoryPlayer();

        void SetInterpolator(std::unique_ptr<TrajectoryInterpolator> interpolator);

        void RecordKeyframe(DebugPlaybackState* playbackState, const std::vector<teleop_viewer::RobotScene::JointInfo>& joints,
                            const teleop_viewer::RobotScene& scene) const;
        void RemoveSelectedKeyframe(DebugPlaybackState* playbackState) const;
        void Clear(DebugPlaybackState* playbackState) const;
        void TogglePlayPause(DebugPlaybackState* playbackState) const;
        void Stop(DebugPlaybackState* playbackState) const;
        void AdvanceAndApply(DebugPlaybackState* playbackState, teleop_viewer::RobotScene* scene, double dtSec) const;
        void SampleAtCurrentTime(const DebugPlaybackState& playbackState, teleop_viewer::RobotScene* scene) const;

        static float TotalDuration(const DebugPlaybackState& playbackState);
        static bool HasPlayableTrajectory(const DebugPlaybackState& playbackState);

       private:
        std::unique_ptr<TrajectoryInterpolator> interpolator_;
    };

    bool LoadTrajectoryFromFile(const std::string& path, DebugPlaybackState* playbackState, std::string* errorMessage);
    bool SaveTrajectoryToFile(const std::string& path, const DebugPlaybackState& playbackState, std::string* errorMessage);
    void BuildDemoTrajectoryFromCurrentPose(DebugPlaybackState* playbackState,
                                            const std::vector<teleop_viewer::RobotScene::JointInfo>& joints,
                                            const teleop_viewer::RobotScene& scene);

}  // namespace kinematic_viewer
