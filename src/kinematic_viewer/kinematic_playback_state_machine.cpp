#include "kinematic_viewer/kinematic_playback_state_machine.h"

#include <algorithm>
#include <cmath>

namespace kinematic_viewer {

    PlaybackStateMachine::PlaybackStateMachine(DebugPlaybackState* state) : state_(state) {}

    PlaybackStateMachine::State PlaybackStateMachine::CurrentState() const {
        if (!state_) {
            return State::Stopped;
        }
        return static_cast<State>(state_->mode);
    }

    bool PlaybackStateMachine::IsPlaying() const {
        return CurrentState() == State::Playing;
    }

    bool PlaybackStateMachine::IsPaused() const {
        return CurrentState() == State::Paused;
    }

    bool PlaybackStateMachine::IsStopped() const {
        return CurrentState() == State::Stopped;
    }

    float PlaybackStateMachine::CurrentTime() const {
        if (!state_) {
            return 0.0f;
        }
        return state_->play_time;
    }

    float PlaybackStateMachine::TotalDuration() const {
        return ComputeTotalDuration();
    }

    bool PlaybackStateMachine::HasKeyframes() const {
        if (!state_) {
            return false;
        }
        return !state_->keyframes.empty();
    }

    bool PlaybackStateMachine::Play() {
        if (!state_ || state_->keyframes.empty()) {
            return false;
        }
        state_->mode = DebugPlaybackState::Mode::Playing;
        return true;
    }

    bool PlaybackStateMachine::Pause() {
        if (!state_ || CurrentState() != State::Playing) {
            return false;
        }
        state_->mode = DebugPlaybackState::Mode::Paused;
        return true;
    }

    bool PlaybackStateMachine::Stop() {
        if (!state_) {
            return false;
        }
        state_->mode      = DebugPlaybackState::Mode::Stopped;
        state_->play_time = 0.0f;
        return true;
    }

    bool PlaybackStateMachine::TogglePlayPause() {
        if (!state_ || state_->keyframes.empty()) {
            return false;
        }
        if (CurrentState() == State::Playing) {
            state_->mode = DebugPlaybackState::Mode::Paused;
        } else {
            state_->mode = DebugPlaybackState::Mode::Playing;
        }
        return true;
    }

    void PlaybackStateMachine::Seek(float time_sec) {
        if (!state_) {
            return;
        }
        const float duration = ComputeTotalDuration();
        state_->play_time    = std::clamp(time_sec, 0.0f, duration);
    }

    void PlaybackStateMachine::AdvanceTime(float dt_sec) {
        if (!state_ || CurrentState() != State::Playing) {
            return;
        }
        const float duration = ComputeTotalDuration();
        if (duration <= 0.0f) {
            return;
        }
        state_->play_time += dt_sec * state_->play_speed;
        if (state_->play_time >= duration) {
            if (state_->loop) {
                state_->play_time = std::fmod(state_->play_time, duration);
            } else {
                state_->play_time = duration;
                state_->mode      = DebugPlaybackState::Mode::Stopped;
            }
        }
    }

    void PlaybackStateMachine::ClampPlayTime() {
        if (!state_) {
            return;
        }
        const float duration = ComputeTotalDuration();
        state_->play_time    = std::clamp(state_->play_time, 0.0f, duration);
    }

    float PlaybackStateMachine::ComputeTotalDuration() const {
        if (!state_ || state_->keyframes.empty()) {
            return 0.0f;
        }
        if (state_->keyframes.size() == 1) {
            return 0.0f;
        }
        return static_cast<float>(state_->keyframes.back().t);
    }

}  // namespace kinematic_viewer
