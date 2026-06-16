#include "kinematic_viewer/kinematic_ui_theme.h"
#include "kinematic_viewer/kinematic_string_utils.h"

#include "imgui.h"

#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace kinematic_viewer {
    namespace kinematic_ui_theme_internal {

        enum ThemeIndex {
            kThemeCurrentDark   = 0,
            kThemeMoonlight     = 1,
            kThemeSpectrumLight = 2,
            kThemeNewton        = 3,
        };

        const std::vector<const char*>& ThemeNamesInternal() {
            static const std::vector<const char*> kNames = {
                "current_dark",
                "moonlight",
                "spectrum_light",
                "newton",
            };
            return kNames;
        }

        static bool FileExists(const std::string& path) {
            std::error_code ec;
            return std::filesystem::exists(path, ec);
        }

        void ApplySharedLayoutStyle() {
            ImGuiStyle& style       = ImGui::GetStyle();
            style.WindowRounding    = 10.0f;
            style.ChildRounding     = 8.0f;
            style.PopupRounding     = 8.0f;
            style.FrameRounding     = 8.0f;
            style.GrabRounding      = 8.0f;
            style.ScrollbarRounding = 10.0f;
            style.TabRounding       = 8.0f;
            style.WindowPadding     = ImVec2(12.0f, 10.0f);
            style.FramePadding      = ImVec2(10.0f, 7.0f);
            style.ItemSpacing       = ImVec2(9.0f, 8.0f);
            style.ItemInnerSpacing  = ImVec2(8.0f, 6.0f);
            style.IndentSpacing     = 18.0f;
            style.WindowBorderSize  = 1.0f;
            style.ChildBorderSize   = 1.0f;
            style.FrameBorderSize   = 1.0f;
            style.ScrollbarSize     = 15.0f;
            style.GrabMinSize       = 12.0f;
        }

        void ApplyCurrentDarkColors() {
            ImVec4* colors                        = ImGui::GetStyle().Colors;
            colors[ImGuiCol_WindowBg]             = ImVec4(0.08f, 0.10f, 0.13f, 1.00f);
            colors[ImGuiCol_ChildBg]              = ImVec4(0.10f, 0.12f, 0.16f, 1.00f);
            colors[ImGuiCol_PopupBg]              = ImVec4(0.11f, 0.13f, 0.17f, 0.98f);
            colors[ImGuiCol_Border]               = ImVec4(0.27f, 0.33f, 0.40f, 0.90f);
            colors[ImGuiCol_BorderShadow]         = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
            colors[ImGuiCol_Text]                 = ImVec4(0.92f, 0.95f, 0.98f, 1.00f);
            colors[ImGuiCol_TextDisabled]         = ImVec4(0.58f, 0.64f, 0.71f, 1.00f);
            colors[ImGuiCol_TitleBg]              = ImVec4(0.10f, 0.12f, 0.16f, 1.00f);
            colors[ImGuiCol_TitleBgActive]        = ImVec4(0.13f, 0.18f, 0.24f, 1.00f);
            colors[ImGuiCol_FrameBg]              = ImVec4(0.14f, 0.18f, 0.24f, 1.00f);
            colors[ImGuiCol_FrameBgHovered]       = ImVec4(0.19f, 0.28f, 0.40f, 1.00f);
            colors[ImGuiCol_FrameBgActive]        = ImVec4(0.22f, 0.35f, 0.50f, 1.00f);
            colors[ImGuiCol_Button]               = ImVec4(0.18f, 0.30f, 0.44f, 0.85f);
            colors[ImGuiCol_ButtonHovered]        = ImVec4(0.23f, 0.41f, 0.60f, 1.00f);
            colors[ImGuiCol_ButtonActive]         = ImVec4(0.28f, 0.49f, 0.70f, 1.00f);
            colors[ImGuiCol_Header]               = ImVec4(0.18f, 0.30f, 0.44f, 0.70f);
            colors[ImGuiCol_HeaderHovered]        = ImVec4(0.23f, 0.41f, 0.60f, 0.88f);
            colors[ImGuiCol_HeaderActive]         = ImVec4(0.28f, 0.49f, 0.70f, 1.00f);
            colors[ImGuiCol_CheckMark]            = ImVec4(0.41f, 0.74f, 1.00f, 1.00f);
            colors[ImGuiCol_SliderGrab]           = ImVec4(0.37f, 0.69f, 0.97f, 1.00f);
            colors[ImGuiCol_SliderGrabActive]     = ImVec4(0.46f, 0.79f, 1.00f, 1.00f);
            colors[ImGuiCol_ResizeGrip]           = ImVec4(0.36f, 0.62f, 0.87f, 0.35f);
            colors[ImGuiCol_ResizeGripHovered]    = ImVec4(0.41f, 0.74f, 1.00f, 0.75f);
            colors[ImGuiCol_ResizeGripActive]     = ImVec4(0.47f, 0.81f, 1.00f, 1.00f);
            colors[ImGuiCol_Separator]            = ImVec4(0.27f, 0.33f, 0.40f, 0.95f);
            colors[ImGuiCol_TableHeaderBg]        = ImVec4(0.13f, 0.19f, 0.27f, 1.00f);
            colors[ImGuiCol_TableBorderStrong]    = ImVec4(0.29f, 0.36f, 0.44f, 1.00f);
            colors[ImGuiCol_TableBorderLight]     = ImVec4(0.20f, 0.26f, 0.33f, 1.00f);
            colors[ImGuiCol_TableRowBg]           = ImVec4(0.10f, 0.12f, 0.16f, 0.45f);
            colors[ImGuiCol_TableRowBgAlt]        = ImVec4(0.12f, 0.15f, 0.20f, 0.65f);
            colors[ImGuiCol_ScrollbarBg]          = ImVec4(0.10f, 0.12f, 0.16f, 1.00f);
            colors[ImGuiCol_ScrollbarGrab]        = ImVec4(0.30f, 0.39f, 0.48f, 0.95f);
            colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.39f, 0.51f, 0.62f, 0.95f);
            colors[ImGuiCol_ScrollbarGrabActive]  = ImVec4(0.49f, 0.63f, 0.75f, 1.00f);
        }

        void ApplyMoonlightColors() {
            ImGui::StyleColorsDark();
            ImVec4* colors                        = ImGui::GetStyle().Colors;
            colors[ImGuiCol_WindowBg]             = ImVec4(0.11f, 0.12f, 0.16f, 1.00f);
            colors[ImGuiCol_ChildBg]              = ImVec4(0.13f, 0.14f, 0.19f, 1.00f);
            colors[ImGuiCol_PopupBg]              = ImVec4(0.12f, 0.13f, 0.18f, 0.98f);
            colors[ImGuiCol_Border]               = ImVec4(0.28f, 0.30f, 0.38f, 0.92f);
            colors[ImGuiCol_Text]                 = ImVec4(0.94f, 0.95f, 0.99f, 1.00f);
            colors[ImGuiCol_TextDisabled]         = ImVec4(0.60f, 0.63f, 0.72f, 1.00f);
            colors[ImGuiCol_TitleBg]              = ImVec4(0.13f, 0.14f, 0.20f, 1.00f);
            colors[ImGuiCol_TitleBgActive]        = ImVec4(0.16f, 0.18f, 0.26f, 1.00f);
            colors[ImGuiCol_FrameBg]              = ImVec4(0.18f, 0.20f, 0.29f, 1.00f);
            colors[ImGuiCol_FrameBgHovered]       = ImVec4(0.26f, 0.31f, 0.45f, 1.00f);
            colors[ImGuiCol_FrameBgActive]        = ImVec4(0.32f, 0.39f, 0.56f, 1.00f);
            colors[ImGuiCol_Button]               = ImVec4(0.29f, 0.35f, 0.50f, 0.90f);
            colors[ImGuiCol_ButtonHovered]        = ImVec4(0.37f, 0.46f, 0.66f, 1.00f);
            colors[ImGuiCol_ButtonActive]         = ImVec4(0.42f, 0.53f, 0.75f, 1.00f);
            colors[ImGuiCol_Header]               = ImVec4(0.28f, 0.34f, 0.49f, 0.80f);
            colors[ImGuiCol_HeaderHovered]        = ImVec4(0.36f, 0.45f, 0.64f, 0.95f);
            colors[ImGuiCol_HeaderActive]         = ImVec4(0.42f, 0.52f, 0.74f, 1.00f);
            colors[ImGuiCol_CheckMark]            = ImVec4(0.76f, 0.81f, 1.00f, 1.00f);
            colors[ImGuiCol_SliderGrab]           = ImVec4(0.66f, 0.74f, 0.99f, 1.00f);
            colors[ImGuiCol_SliderGrabActive]     = ImVec4(0.77f, 0.83f, 1.00f, 1.00f);
            colors[ImGuiCol_ResizeGrip]           = ImVec4(0.44f, 0.52f, 0.74f, 0.38f);
            colors[ImGuiCol_ResizeGripHovered]    = ImVec4(0.53f, 0.63f, 0.86f, 0.75f);
            colors[ImGuiCol_ResizeGripActive]     = ImVec4(0.60f, 0.72f, 0.95f, 1.00f);
            colors[ImGuiCol_Separator]            = ImVec4(0.28f, 0.31f, 0.39f, 0.95f);
            colors[ImGuiCol_TableHeaderBg]        = ImVec4(0.19f, 0.22f, 0.31f, 1.00f);
            colors[ImGuiCol_TableBorderStrong]    = ImVec4(0.32f, 0.36f, 0.45f, 1.00f);
            colors[ImGuiCol_TableBorderLight]     = ImVec4(0.23f, 0.27f, 0.36f, 1.00f);
            colors[ImGuiCol_TableRowBg]           = ImVec4(0.12f, 0.13f, 0.18f, 0.45f);
            colors[ImGuiCol_TableRowBgAlt]        = ImVec4(0.15f, 0.16f, 0.22f, 0.60f);
            colors[ImGuiCol_ScrollbarBg]          = ImVec4(0.12f, 0.13f, 0.18f, 1.00f);
            colors[ImGuiCol_ScrollbarGrab]        = ImVec4(0.35f, 0.40f, 0.53f, 0.95f);
            colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.45f, 0.53f, 0.69f, 0.98f);
            colors[ImGuiCol_ScrollbarGrabActive]  = ImVec4(0.54f, 0.64f, 0.82f, 1.00f);
        }

        void ApplySpectrumLightColors() {
            ImGui::StyleColorsLight();
            ImVec4* colors                        = ImGui::GetStyle().Colors;
            colors[ImGuiCol_WindowBg]             = ImVec4(0.95f, 0.95f, 0.96f, 1.00f);
            colors[ImGuiCol_ChildBg]              = ImVec4(0.98f, 0.98f, 0.99f, 1.00f);
            colors[ImGuiCol_PopupBg]              = ImVec4(1.00f, 1.00f, 1.00f, 0.98f);
            colors[ImGuiCol_Border]               = ImVec4(0.73f, 0.74f, 0.77f, 0.95f);
            colors[ImGuiCol_Text]                 = ImVec4(0.18f, 0.20f, 0.24f, 1.00f);
            colors[ImGuiCol_TextDisabled]         = ImVec4(0.47f, 0.50f, 0.56f, 1.00f);
            colors[ImGuiCol_TitleBg]              = ImVec4(0.90f, 0.91f, 0.93f, 1.00f);
            colors[ImGuiCol_TitleBgActive]        = ImVec4(0.86f, 0.88f, 0.92f, 1.00f);
            colors[ImGuiCol_FrameBg]              = ImVec4(0.90f, 0.92f, 0.95f, 1.00f);
            colors[ImGuiCol_FrameBgHovered]       = ImVec4(0.82f, 0.88f, 0.97f, 1.00f);
            colors[ImGuiCol_FrameBgActive]        = ImVec4(0.74f, 0.84f, 0.97f, 1.00f);
            colors[ImGuiCol_Button]               = ImVec4(0.31f, 0.51f, 0.82f, 0.88f);
            colors[ImGuiCol_ButtonHovered]        = ImVec4(0.23f, 0.45f, 0.78f, 0.95f);
            colors[ImGuiCol_ButtonActive]         = ImVec4(0.18f, 0.39f, 0.71f, 1.00f);
            colors[ImGuiCol_Header]               = ImVec4(0.77f, 0.86f, 0.98f, 0.85f);
            colors[ImGuiCol_HeaderHovered]        = ImVec4(0.67f, 0.80f, 0.96f, 0.92f);
            colors[ImGuiCol_HeaderActive]         = ImVec4(0.58f, 0.75f, 0.94f, 1.00f);
            colors[ImGuiCol_CheckMark]            = ImVec4(0.20f, 0.44f, 0.76f, 1.00f);
            colors[ImGuiCol_SliderGrab]           = ImVec4(0.24f, 0.47f, 0.80f, 1.00f);
            colors[ImGuiCol_SliderGrabActive]     = ImVec4(0.17f, 0.38f, 0.69f, 1.00f);
            colors[ImGuiCol_ResizeGrip]           = ImVec4(0.35f, 0.55f, 0.86f, 0.30f);
            colors[ImGuiCol_ResizeGripHovered]    = ImVec4(0.28f, 0.49f, 0.81f, 0.70f);
            colors[ImGuiCol_ResizeGripActive]     = ImVec4(0.20f, 0.43f, 0.74f, 0.95f);
            colors[ImGuiCol_Separator]            = ImVec4(0.72f, 0.74f, 0.78f, 0.95f);
            colors[ImGuiCol_TableHeaderBg]        = ImVec4(0.86f, 0.90f, 0.96f, 1.00f);
            colors[ImGuiCol_TableBorderStrong]    = ImVec4(0.68f, 0.71f, 0.77f, 1.00f);
            colors[ImGuiCol_TableBorderLight]     = ImVec4(0.80f, 0.83f, 0.88f, 1.00f);
            colors[ImGuiCol_TableRowBg]           = ImVec4(0.96f, 0.97f, 0.99f, 0.35f);
            colors[ImGuiCol_TableRowBgAlt]        = ImVec4(0.92f, 0.95f, 0.99f, 0.55f);
            colors[ImGuiCol_ScrollbarBg]          = ImVec4(0.92f, 0.93f, 0.96f, 1.00f);
            colors[ImGuiCol_ScrollbarGrab]        = ImVec4(0.65f, 0.70f, 0.78f, 0.95f);
            colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.54f, 0.62f, 0.73f, 0.98f);
            colors[ImGuiCol_ScrollbarGrabActive]  = ImVec4(0.43f, 0.54f, 0.68f, 1.00f);
        }

        void ApplyNewtonColors() {
            ImGui::StyleColorsDark();
            ImVec4* colors                        = ImGui::GetStyle().Colors;
            colors[ImGuiCol_WindowBg]             = ImVec4(0.10f, 0.11f, 0.14f, 0.92f);
            colors[ImGuiCol_ChildBg]              = ImVec4(0.12f, 0.13f, 0.17f, 0.95f);
            colors[ImGuiCol_PopupBg]              = ImVec4(0.11f, 0.12f, 0.16f, 0.96f);
            colors[ImGuiCol_Border]               = ImVec4(0.28f, 0.34f, 0.44f, 0.75f);
            colors[ImGuiCol_Text]                 = ImVec4(0.93f, 0.95f, 0.98f, 1.00f);
            colors[ImGuiCol_TextDisabled]         = ImVec4(0.55f, 0.62f, 0.72f, 1.00f);
            colors[ImGuiCol_TitleBg]              = ImVec4(0.10f, 0.11f, 0.15f, 1.00f);
            colors[ImGuiCol_TitleBgActive]        = ImVec4(0.14f, 0.17f, 0.23f, 1.00f);
            colors[ImGuiCol_FrameBg]              = ImVec4(0.16f, 0.19f, 0.26f, 1.00f);
            colors[ImGuiCol_FrameBgHovered]       = ImVec4(0.20f, 0.26f, 0.36f, 1.00f);
            colors[ImGuiCol_FrameBgActive]        = ImVec4(0.24f, 0.32f, 0.44f, 1.00f);
            colors[ImGuiCol_Button]               = ImVec4(0.18f, 0.28f, 0.42f, 0.88f);
            colors[ImGuiCol_ButtonHovered]        = ImVec4(0.24f, 0.38f, 0.56f, 1.00f);
            colors[ImGuiCol_ButtonActive]         = ImVec4(0.30f, 0.48f, 0.68f, 1.00f);
            colors[ImGuiCol_Header]               = ImVec4(0.18f, 0.28f, 0.42f, 0.72f);
            colors[ImGuiCol_HeaderHovered]        = ImVec4(0.24f, 0.38f, 0.56f, 0.90f);
            colors[ImGuiCol_HeaderActive]         = ImVec4(0.30f, 0.48f, 0.68f, 1.00f);
            colors[ImGuiCol_CheckMark]            = ImVec4(0.60f, 0.80f, 1.00f, 1.00f);
            colors[ImGuiCol_SliderGrab]           = ImVec4(0.45f, 0.68f, 0.94f, 1.00f);
            colors[ImGuiCol_SliderGrabActive]     = ImVec4(0.60f, 0.80f, 1.00f, 1.00f);
            colors[ImGuiCol_ResizeGrip]           = ImVec4(0.36f, 0.54f, 0.78f, 0.35f);
            colors[ImGuiCol_ResizeGripHovered]    = ImVec4(0.45f, 0.68f, 0.94f, 0.75f);
            colors[ImGuiCol_ResizeGripActive]     = ImVec4(0.60f, 0.80f, 1.00f, 1.00f);
            colors[ImGuiCol_Separator]            = ImVec4(0.28f, 0.34f, 0.44f, 0.90f);
            colors[ImGuiCol_TableHeaderBg]        = ImVec4(0.14f, 0.17f, 0.23f, 1.00f);
            colors[ImGuiCol_TableBorderStrong]    = ImVec4(0.30f, 0.36f, 0.46f, 1.00f);
            colors[ImGuiCol_TableBorderLight]     = ImVec4(0.22f, 0.27f, 0.35f, 1.00f);
            colors[ImGuiCol_TableRowBg]           = ImVec4(0.11f, 0.12f, 0.16f, 0.45f);
            colors[ImGuiCol_TableRowBgAlt]        = ImVec4(0.14f, 0.16f, 0.21f, 0.65f);
            colors[ImGuiCol_ScrollbarBg]          = ImVec4(0.10f, 0.11f, 0.14f, 1.00f);
            colors[ImGuiCol_ScrollbarGrab]        = ImVec4(0.30f, 0.38f, 0.50f, 0.95f);
            colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.40f, 0.52f, 0.68f, 0.98f);
            colors[ImGuiCol_ScrollbarGrabActive]  = ImVec4(0.50f, 0.64f, 0.82f, 1.00f);
        }

    }  // namespace kinematic_ui_theme_internal

    void SetupKinematicViewerFonts(const KinematicViewerConfig& cfg) {
        ImGuiIO& io           = ImGui::GetIO();
        float font_size       = std::max(12.0f, cfg.ui.cjk_font_size);
        const ImWchar* ranges = io.Fonts->GetGlyphRangesChineseFull();

        std::string loaded_font_path;
        ImFontConfig font_cfg;
        font_cfg.OversampleH = 2;
        font_cfg.OversampleV = 1;
        font_cfg.PixelSnapH  = true;

        std::vector<std::string> font_candidates;
        if (!cfg.ui.cjk_font_path.empty()) {
            font_candidates.push_back(cfg.ui.cjk_font_path);
        }
        font_candidates.push_back("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc");
        font_candidates.push_back("/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc");
        font_candidates.push_back("/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf");
        font_candidates.push_back("/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc");
        font_candidates.push_back("/usr/share/fonts/truetype/wqy/wqy-microhei.ttc");

        for (const auto& path : font_candidates) {
            if (!kinematic_ui_theme_internal::FileExists(path)) {
                continue;
            }
            if (io.Fonts->AddFontFromFileTTF(path.c_str(), font_size, &font_cfg, ranges)) {
                loaded_font_path = path;
                std::cout << "[robot_kinematic_viewer] Loaded CJK font: " << loaded_font_path << " (size=" << font_size << ")" << std::endl;
                return;
            }
        }
        io.Fonts->AddFontDefault();
        std::cerr << "[robot_kinematic_viewer] No CJK font found. Chinese text may show as '?'. "
                  << "Please set ui.cjk_font_path in config." << std::endl;
    }

    const std::vector<const char*>& KinematicUiThemeNames() {
        return kinematic_ui_theme_internal::ThemeNamesInternal();
    }

    int KinematicUiThemeIndexFromName(const std::string& themeName) {
        const std::string target = kinematic_viewer::LowerString(themeName);
        const auto& names        = KinematicUiThemeNames();
        for (int i = 0; i < static_cast<int>(names.size()); ++i) {
            if (target == names[static_cast<size_t>(i)]) {
                return i;
            }
        }
        return kinematic_ui_theme_internal::kThemeCurrentDark;
    }

    void ApplyKinematicUiStyleByIndex(int themeIndex) {
        using namespace kinematic_ui_theme_internal;
        if (themeIndex == kThemeSpectrumLight) {
            ApplySpectrumLightColors();
        } else if (themeIndex == kThemeMoonlight) {
            ApplyMoonlightColors();
        } else if (themeIndex == kThemeNewton) {
            ApplyNewtonColors();
        } else {
            ImGui::StyleColorsDark();
            ApplyCurrentDarkColors();
        }
        ApplySharedLayoutStyle();
    }

    void ApplyKinematicUiStyleByName(const std::string& themeName) {
        ApplyKinematicUiStyleByIndex(KinematicUiThemeIndexFromName(themeName));
    }

}  // namespace kinematic_viewer
