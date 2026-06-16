#pragma once

#include "kinematic_viewer/rkv_panel_plugin.h"

#include <string>
#include <vector>

namespace kinematic_viewer {

    // One loaded panel entry.
    struct RkvPanelEntry {
        std::string id;
        std::string label;
        void* dl_handle            = nullptr;  // from dlopen; nullptr for statically-linked stubs
        RkvPanelInfoFn info_fn     = nullptr;
        RkvPanelRenderFn render_fn = nullptr;
    };

    // Loads panel .so files by path or by built-in id, and provides rendering.
    // Usage:
    //   RkvPanelRegistry reg;
    //   reg.LoadFromConfig(panel_specs, search_dirs);
    //   ...
    //   reg.Render(page_index, &ctx);
    class RkvPanelRegistry {
       public:
        ~RkvPanelRegistry();

        // Load panels in order from a list of specs.
        // Each spec is either:
        //   - a built-in id ("scene", "joint", …)
        //   - an absolute path to a .so
        //   - a filename relative to one of search_dirs
        // Unknown ids are skipped with a warning; duplicates are ignored.
        void LoadFromConfig(const std::vector<std::string>& specs, const std::vector<std::string>& search_dirs);

        // Number of active panels.
        int Count() const { return static_cast<int>(panels_.size()); }

        // Tab label for panel i.
        const std::string& Label(int i) const;

        // Id for panel i.
        const std::string& Id(int i) const;

        // Call render_fn for the active panel.
        void Render(int page_index, RkvPanelCtx* ctx) const;

        // Whether the panel with given id is registered.
        bool HasPanel(const std::string& id) const;

        // Index of panel with given id, or -1.
        int IndexOf(const std::string& id) const;

       private:
        void LoadPlugin(const std::string& so_path);
        void LoadBuiltin(const std::string& id);

        std::vector<RkvPanelEntry> panels_;
        static const std::string kEmptyStr;
    };

}  // namespace kinematic_viewer
