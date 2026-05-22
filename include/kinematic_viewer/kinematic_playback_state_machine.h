#pragma once

#include "kinematic_viewer/kinematic_playback_state.h"

#include <string>

namespace kinematic_viewer {

    // Encapsulates playback state machine logic, providing explicit state transitions
    // and guarding against invalid transitions. Replaces scattered state mutation in
    // kinematic_playback.cpp and robot_kinematic_viewer.cpp.
    class PlaybackStateMachine {
       public:
        enum class State {
            Stopped,
            Playing,
            Paused,
        };

        explicit PlaybackStateMachine(DebugPlaybackState* state);

        // State transitions
        bool Play();
        bool Pause();
        bool Stop();
        bool TogglePlayPause();

        // Seek to a specific time (clamped to [0, duration]).
        void Seek(float time_sec);

        // Advance playback time by dt (only when Playing).
        void AdvanceTime(float dt_sec);

        // Query
        State CurrentState() const;
        bool IsPlaying() const;
        bool IsPaused() const;
        bool IsStopped() const;
        float CurrentTime() const;
        float TotalDuration() const;
        bool HasKeyframes() const;

        // Access the underlying state (for backward compatibility with existing panels).
        DebugPlaybackState* GetState() const { return state_; }

       private:
        DebugPlaybackState* state_ = nullptr;

        void ClampPlayTime();
        float ComputeTotalDuration() const;
    };

}  // namespace kinematic_viewer
