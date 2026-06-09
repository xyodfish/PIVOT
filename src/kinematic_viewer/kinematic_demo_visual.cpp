#include "kinematic_viewer/kinematic_demo_visual.h"

namespace kinematic_viewer {

    void SetDemoVisualMode(ViewerState* ui_state, bool enabled) {
        if (ui_state == nullptr || ui_state->demo_visual_mode == enabled) {
            return;
        }

        if (enabled) {
            ui_state->demo_visual_saved                 = true;
            ui_state->saved_show_axes                   = ui_state->show_axes;
            ui_state->saved_show_world_axes             = ui_state->show_world_axes;
            ui_state->saved_show_collision_bodies       = ui_state->show_collision_bodies;
            ui_state->saved_show_com                    = ui_state->show_com;
            ui_state->saved_show_grid                   = ui_state->show_grid;
            ui_state->saved_enable_link_hover_highlight = ui_state->enable_link_hover_highlight;

            ui_state->show_axes                   = false;
            ui_state->show_world_axes             = false;
            ui_state->show_collision_bodies       = false;
            ui_state->show_com                    = false;
            ui_state->show_grid                   = false;
            ui_state->enable_link_hover_highlight = false;
        } else if (ui_state->demo_visual_saved) {
            ui_state->show_axes                   = ui_state->saved_show_axes;
            ui_state->show_world_axes             = ui_state->saved_show_world_axes;
            ui_state->show_collision_bodies       = ui_state->saved_show_collision_bodies;
            ui_state->show_com                    = ui_state->saved_show_com;
            ui_state->show_grid                   = ui_state->saved_show_grid;
            ui_state->enable_link_hover_highlight = ui_state->saved_enable_link_hover_highlight;
        }

        ui_state->demo_visual_mode = enabled;
    }

    void ToggleDemoVisualMode(ViewerState* ui_state) {
        if (ui_state == nullptr) {
            return;
        }
        SetDemoVisualMode(ui_state, !ui_state->demo_visual_mode);
    }

}  // namespace kinematic_viewer
