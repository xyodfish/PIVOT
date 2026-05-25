#pragma once

#include "imgui.h"

#include <algorithm>
#include <cmath>
#include <string>

namespace kinematic_viewer {

    inline std::string PathBasename(const std::string& path) {
        if (path.empty()) {
            return "";
        }
        const size_t slash = path.find_last_of('/');
        return (slash == std::string::npos) ? path : path.substr(slash + 1);
    }

    inline bool SidebarPageShowsLinkInspector(int sidebar_page) {
        return sidebar_page == 0 || sidebar_page == 4 || sidebar_page == 5;
    }

    inline ImVec2 SidebarListSize(float preferred_height, float min_height = 100.0f) {
        const float h = std::max(min_height, preferred_height);
        return ImVec2(0.0f, h);
    }

    inline void BeginSidebarScrollRegion(const char* id) {
        const float h = std::max(80.0f, ImGui::GetContentRegionAvail().y);
        ImGui::BeginChild(id, ImVec2(0.0f, h), false, ImGuiWindowFlags_AlwaysVerticalScrollbar);
    }

    inline void EndSidebarScrollRegion() {
        ImGui::EndChild();
    }

    inline float SidebarAvailWidth() {
        return std::max(120.0f, ImGui::GetContentRegionAvail().x);
    }

    inline float SidebarHalfWidth() {
        const float spacing = ImGui::GetStyle().ItemSpacing.x;
        return std::max(72.0f, (SidebarAvailWidth() - spacing) * 0.5f);
    }

    inline void PushSidebarFullWidth() {
        ImGui::PushItemWidth(-1.0f);
    }

    inline void PushSidebarHalfWidth() {
        ImGui::PushItemWidth(SidebarHalfWidth());
    }

    inline void PopSidebarWidth() {
        ImGui::PopItemWidth();
    }

    inline bool SidebarCombo(const char* label, int* current, const char* const items[], int count) {
        PushSidebarFullWidth();
        const bool changed = ImGui::Combo(label, current, items, count);
        PopSidebarWidth();
        return changed;
    }

    inline bool SidebarDragFloat(const char* label, float* v, float speed, float min_v, float max_v, const char* fmt) {
        PushSidebarFullWidth();
        const bool changed = ImGui::DragFloat(label, v, speed, min_v, max_v, fmt);
        PopSidebarWidth();
        return changed;
    }

    inline bool SidebarSliderFloat(const char* label, float* v, float min_v, float max_v, const char* fmt) {
        PushSidebarFullWidth();
        const bool changed = ImGui::SliderFloat(label, v, min_v, max_v, fmt);
        PopSidebarWidth();
        return changed;
    }

    inline void SidebarCheckboxRow2(const char* l1, bool* v1, const char* l2, bool* v2) {
        ImGui::Checkbox(l1, v1);
        ImGui::SameLine(SidebarHalfWidth() + ImGui::GetStyle().ItemSpacing.x);
        ImGui::Checkbox(l2, v2);
    }

    inline void SidebarCheckboxRow4(const char* l1, bool* v1, const char* l2, bool* v2, const char* l3, bool* v3, const char* l4,
                                  bool* v4) {
        const float col_w = (SidebarAvailWidth() - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
        ImGui::Checkbox(l1, v1);
        ImGui::SameLine(col_w);
        ImGui::Checkbox(l2, v2);
        ImGui::Checkbox(l3, v3);
        ImGui::SameLine(col_w);
        ImGui::Checkbox(l4, v4);
    }

    inline void SidebarInputText(const char* label, char* buf, size_t buf_size) {
        PushSidebarFullWidth();
        ImGui::InputText(label, buf, buf_size);
        PopSidebarWidth();
    }

    inline void SidebarInputTextMultiline(const char* label, char* buf, size_t buf_size, float height) {
        PushSidebarFullWidth();
        ImGui::InputTextMultiline(label, buf, buf_size, ImVec2(0.0f, height));
        PopSidebarWidth();
    }

    inline void SidebarRadioRow3(const char* label, int* v, const char* o0, const char* o1, const char* o2) {
        ImGui::TextUnformatted(label);
        ImGui::SameLine();
        ImGui::RadioButton(o0, v, 0);
        ImGui::SameLine();
        ImGui::RadioButton(o1, v, 1);
        ImGui::SameLine();
        ImGui::RadioButton(o2, v, 2);
    }

}  // namespace kinematic_viewer
