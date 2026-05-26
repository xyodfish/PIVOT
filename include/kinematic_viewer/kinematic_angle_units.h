#pragma once

#include "imgui.h"

#include <glm/gtc/constants.hpp>

namespace kinematic_viewer {

    inline const char* AngleUnitLabel(bool angle_unit_deg) {
        return angle_unit_deg ? "deg" : "rad";
    }

    inline const char* AngleInputFormat(bool angle_unit_deg) {
        return angle_unit_deg ? "%.2f" : "%.4f";
    }

    inline float AngleUiFromRad(float rad, bool angle_unit_deg) {
        return angle_unit_deg ? glm::degrees(rad) : rad;
    }

    inline float AngleUiToRad(float ui_value, bool angle_unit_deg) {
        return angle_unit_deg ? glm::radians(ui_value) : ui_value;
    }

    // Fields stored in degrees (markerWorldMatrix, rpy_deg, etc.).
    inline float AngleUiFromDegStored(float deg_stored, bool angle_unit_deg) {
        return angle_unit_deg ? deg_stored : glm::radians(deg_stored);
    }

    inline float AngleUiToDegStored(float ui_value, bool angle_unit_deg) {
        return angle_unit_deg ? ui_value : glm::degrees(ui_value);
    }

    inline glm::vec3 RpyUiFromDegStored(const glm::vec3& rpy_deg, bool angle_unit_deg) {
        return glm::vec3(AngleUiFromDegStored(rpy_deg.x, angle_unit_deg), AngleUiFromDegStored(rpy_deg.y, angle_unit_deg),
                         AngleUiFromDegStored(rpy_deg.z, angle_unit_deg));
    }

    inline glm::vec3 RpyUiToDegStored(const glm::vec3& rpy_ui, bool angle_unit_deg) {
        return glm::vec3(AngleUiToDegStored(rpy_ui.x, angle_unit_deg), AngleUiToDegStored(rpy_ui.y, angle_unit_deg),
                         AngleUiToDegStored(rpy_ui.z, angle_unit_deg));
    }

    inline float AngleDragMin(bool angle_unit_deg) {
        return angle_unit_deg ? -360.0f : -glm::pi<float>() * 2.0f;
    }

    inline float AngleDragMax(bool angle_unit_deg) {
        return angle_unit_deg ? 360.0f : glm::pi<float>() * 2.0f;
    }

    inline void RenderAngleUnitSelector(bool* angle_unit_deg) {
        if (angle_unit_deg == nullptr) {
            return;
        }
        int unit = *angle_unit_deg ? 1 : 0;
        ImGui::TextUnformatted("角度单位");
        ImGui::SameLine();
        ImGui::RadioButton("deg##global_angle_unit", &unit, 1);
        ImGui::SameLine();
        ImGui::RadioButton("rad##global_angle_unit", &unit, 0);
        *angle_unit_deg = (unit == 1);
    }

}  // namespace kinematic_viewer
