#include "kinematic_viewer/rkv_panel_registry.h"

#include <dlfcn.h>
#include <unistd.h>

#include <filesystem>
#include <iostream>

#include <glog/logging.h>

namespace kinematic_viewer {

    const std::string RkvPanelRegistry::kEmptyStr;

    RkvPanelRegistry::~RkvPanelRegistry() {
        for (auto& p : panels_) {
            if (p.dl_handle) {
                dlclose(p.dl_handle);
            }
        }
    }

    // ---------------------------------------------------------------------------
    // internal helpers
    // ---------------------------------------------------------------------------

    static bool idAlreadyLoaded(const std::vector<RkvPanelEntry>& panels, const std::string& id) {
        for (const auto& p : panels) {
            if (p.id == id) {
                return true;
            }
        }
        return false;
    }

    void RkvPanelRegistry::LoadPlugin(const std::string& so_path) {
        void* handle = dlopen(so_path.c_str(), RTLD_NOW | RTLD_LOCAL);
        if (!handle) {
            std::cerr << "[RkvPanelRegistry] dlopen failed: " << so_path << " — " << dlerror() << "\n";
            return;
        }

        auto* info_fn   = reinterpret_cast<RkvPanelInfoFn>(dlsym(handle, "rkv_panel_info"));
        auto* render_fn = reinterpret_cast<RkvPanelRenderFn>(dlsym(handle, "rkv_panel_render"));
        if (!info_fn || !render_fn) {
            std::cerr << "[RkvPanelRegistry] missing symbols in: " << so_path << " (need rkv_panel_info + rkv_panel_render)\n";
            dlclose(handle);
            return;
        }

        RkvPanelInfo info{};
        info_fn(&info);
        const std::string id    = info.id ? info.id : "";
        const std::string label = info.label ? info.label : id;

        if (id.empty()) {
            std::cerr << "[RkvPanelRegistry] panel returned empty id: " << so_path << "\n";
            dlclose(handle);
            return;
        }

        if (idAlreadyLoaded(panels_, id)) {
            std::cerr << "[RkvPanelRegistry] duplicate panel id '" << id << "', skipping: " << so_path << "\n";
            dlclose(handle);
            return;
        }

        RkvPanelEntry entry;
        entry.id        = id;
        entry.label     = label;
        entry.dl_handle = handle;
        entry.info_fn   = info_fn;
        entry.render_fn = render_fn;
        panels_.push_back(std::move(entry));
        LOG(INFO) << "loaded panel '" << id << "' from " << so_path;
    }

    // Built-in panel ids map to lib/librkv_panel_<id>.so beside the executable.
    void RkvPanelRegistry::LoadBuiltin(const std::string& id) {
        // Find the executable directory at runtime.
        std::string exe_dir;
        {
            char buf[4096] = {};
            ssize_t len    = ::readlink("/proc/self/exe", buf, sizeof(buf) - 1);
            if (len > 0) {
                buf[len] = '\0';
                exe_dir  = std::filesystem::path(buf).parent_path().string();
            }
        }

        // Try <exe_dir>/../lib/librkv_panel_<id>.so, then <exe_dir>/lib/...
        const std::string so_name                 = "librkv_panel_" + id + ".so";
        const std::vector<std::string> candidates = {
            exe_dir + "/../lib/" + so_name,
            exe_dir + "/lib/" + so_name,
            exe_dir + "/" + so_name,
        };
        for (const auto& candidate : candidates) {
            std::error_code ec;
            if (std::filesystem::exists(candidate, ec)) {
                LoadPlugin(candidate);
                return;
            }
        }
        std::cerr << "[RkvPanelRegistry] built-in panel '" << id << "': could not find " << so_name << "\n";
    }

    // ---------------------------------------------------------------------------
    // public
    // ---------------------------------------------------------------------------

    void RkvPanelRegistry::LoadFromConfig(const std::vector<std::string>& specs, const std::vector<std::string>& search_dirs) {
        for (const auto& spec : specs) {
            if (spec.empty()) {
                continue;
            }

            // Absolute path or explicit .so extension → load directly.
            const bool is_path = (spec[0] == '/' || spec[0] == '.' || spec.size() > 3 && spec.substr(spec.size() - 3) == ".so");
            if (is_path) {
                if (std::filesystem::exists(spec)) {
                    LoadPlugin(spec);
                } else {
                    // Try search_dirs.
                    bool found = false;
                    for (const auto& dir : search_dirs) {
                        const std::string full = dir + "/" + spec;
                        if (std::filesystem::exists(full)) {
                            LoadPlugin(full);
                            found = true;
                            break;
                        }
                    }
                    if (!found) {
                        std::cerr << "[RkvPanelRegistry] panel so not found: " << spec << "\n";
                    }
                }
            } else {
                // Treat as built-in id.
                LoadBuiltin(spec);
            }
        }
    }

    const std::string& RkvPanelRegistry::Label(int i) const {
        if (i < 0 || i >= static_cast<int>(panels_.size())) {
            return kEmptyStr;
        }
        return panels_[static_cast<size_t>(i)].label;
    }

    const std::string& RkvPanelRegistry::Id(int i) const {
        if (i < 0 || i >= static_cast<int>(panels_.size())) {
            return kEmptyStr;
        }
        return panels_[static_cast<size_t>(i)].id;
    }

    void RkvPanelRegistry::Render(int page_index, RkvPanelCtx* ctx) const {
        if (page_index < 0 || page_index >= static_cast<int>(panels_.size())) {
            return;
        }
        if (panels_[static_cast<size_t>(page_index)].render_fn) {
            panels_[static_cast<size_t>(page_index)].render_fn(ctx);
        }
    }

    bool RkvPanelRegistry::HasPanel(const std::string& id) const {
        return IndexOf(id) >= 0;
    }

    int RkvPanelRegistry::IndexOf(const std::string& id) const {
        for (int i = 0; i < static_cast<int>(panels_.size()); ++i) {
            if (panels_[static_cast<size_t>(i)].id == id) {
                return i;
            }
        }
        return -1;
    }

}  // namespace kinematic_viewer
