#include "kinematic_viewer/kinematic_point_cloud.h"

#include "kinematic_viewer/kinematic_point_cloud_pcl.h"

#include "imgui.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <dlfcn.h>
#include <limits.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <sstream>
#include <string>
#include <vector>

namespace kinematic_viewer {
    namespace {

        using LoadFn = int (*)(const char*, const RkvPcdLoadParams*, float**, float**, size_t*, size_t*, char*, size_t);
        using EsdfFn = int (*)(const char*, const RkvPcdLoadParams*, const RkvEsdfBuildParams*, float**, float**, size_t*, size_t*, char*,
                               size_t);
        using FreeFn = void (*)(float*);

        struct PclPlugin {
            void* handle      = nullptr;
            LoadFn load       = nullptr;
            EsdfFn build_esdf = nullptr;
            FreeFn free_fn    = nullptr;
            std::string last_error;

            ~PclPlugin() {
                if (handle != nullptr) {
                    dlclose(handle);
                    handle = nullptr;
                }
            }

            bool ensureLoaded() {
                if (load != nullptr && free_fn != nullptr) {
                    return true;
                }
                std::vector<std::string> candidates;
                if (const char* override_path = std::getenv("RKV_PCL_PLUGIN")) {
                    candidates.emplace_back(override_path);
                }
                char exe_path[PATH_MAX] = {};
                if (readlink("/proc/self/exe", exe_path, sizeof(exe_path) - 1) > 0) {
                    std::string base = exe_path;
                    const auto pos   = base.find_last_of('/');
                    if (pos != std::string::npos) {
                        const std::string dir = base.substr(0, pos);
                        candidates.emplace_back(dir + "/../lib/librkv_point_cloud_pcl.so");
                        candidates.emplace_back(dir + "/librkv_point_cloud_pcl.so");
                    }
                }
                candidates.emplace_back("librkv_point_cloud_pcl.so");

                for (const auto& path : candidates) {
                    handle = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
                    if (handle != nullptr) {
                        load       = reinterpret_cast<LoadFn>(dlsym(handle, "rkv_pcd_load_file"));
                        build_esdf = reinterpret_cast<EsdfFn>(dlsym(handle, "rkv_esdf_build_from_pcd"));
                        free_fn    = reinterpret_cast<FreeFn>(dlsym(handle, "rkv_pcd_free"));
                        if (load != nullptr && free_fn != nullptr) {
                            return true;
                        }
                        last_error = std::string("invalid plugin symbols: ") + path;
                        dlclose(handle);
                        handle     = nullptr;
                        load       = nullptr;
                        build_esdf = nullptr;
                        free_fn    = nullptr;
                        continue;
                    }
                }
                last_error = std::string("dlopen librkv_point_cloud_pcl.so failed: ") + dlerror();
                return false;
            }
        };

        PclPlugin& plugin() {
            static PclPlugin instance;
            return instance;
        }

        RkvPcdLoadParams toCParams(const PointCloudLoadOptions& opt) {
            RkvPcdLoadParams p{};
            p.voxel_size = opt.voxel_size;
            p.max_points = opt.max_points;
            p.x_min      = opt.x_min;
            p.x_max      = opt.x_max;
            p.y_min      = opt.y_min;
            p.y_max      = opt.y_max;
            p.z_min      = opt.z_min;
            p.z_max      = opt.z_max;
            p.color_mode = static_cast<int>(opt.color_mode);
            return p;
        }

        RkvEsdfBuildParams toEsdfParams(const PointCloudLoadOptions& opt) {
            RkvEsdfBuildParams p{};
            p.esdf_resolution = opt.esdf_resolution;
            p.map_origin[0]   = opt.esdf_map_origin[0];
            p.map_origin[1]   = opt.esdf_map_origin[1];
            p.map_origin[2]   = opt.esdf_map_origin[2];
            p.map_size[0]     = opt.esdf_map_size[0];
            p.map_size[1]     = opt.esdf_map_size[1];
            p.map_size[2]     = opt.esdf_map_size[2];
            p.use_fixed_map   = opt.esdf_use_fixed_map ? 1 : 0;
            p.padding         = static_cast<float>(opt.esdf_resolution);
            p.x_min           = opt.x_min;
            p.x_max           = opt.x_max;
            p.y_min           = opt.y_min;
            p.y_max           = opt.y_max;
            p.z_min           = opt.z_min;
            p.z_max           = opt.z_max;
            p.max_visual_dist = opt.esdf_max_visual_dist;
            p.visual_stride   = opt.esdf_visual_stride;
            p.max_points      = opt.max_points;
            p.visual_mode     = opt.esdf_visual_mode;
            p.color_mode      = opt.esdf_color_mode;
            p.z_slice_enable  = opt.esdf_z_slice_enable ? 1 : 0;
            p.z_slice_m       = opt.esdf_z_slice_m;
            p.use_raycast     = opt.esdf_use_raycast ? 1 : 0;
            p.ray_origin[0]   = opt.esdf_ray_origin[0];
            p.ray_origin[1]   = opt.esdf_ray_origin[1];
            p.ray_origin[2]   = opt.esdf_ray_origin[2];
            p.ray_origin_auto = opt.esdf_ray_origin_auto ? 1 : 0;
            p.min_ray_length  = opt.esdf_min_ray_length;
            p.max_ray_length  = opt.esdf_max_ray_length;
            return p;
        }

        PointCloudLoadOptions optionsFromUiBase(const PointCloudUiState& ui);

        PointCloudLoadOptions optionsFromUi(const PointCloudUiState& ui) {
            PointCloudLoadOptions opt = optionsFromUiBase(ui);
            opt.build_esdf            = ui.build_esdf;
            opt.esdf_resolution       = ui.esdf_resolution;
            opt.esdf_map_origin[0]    = ui.esdf_map_origin[0];
            opt.esdf_map_origin[1]    = ui.esdf_map_origin[1];
            opt.esdf_map_origin[2]    = ui.esdf_map_origin[2];
            opt.esdf_map_size[0]      = ui.esdf_map_size[0];
            opt.esdf_map_size[1]      = ui.esdf_map_size[1];
            opt.esdf_map_size[2]      = ui.esdf_map_size[2];
            opt.esdf_use_fixed_map    = ui.esdf_use_fixed_map;
            opt.esdf_max_visual_dist  = ui.esdf_max_visual_dist;
            opt.esdf_visual_stride    = ui.esdf_visual_stride;
            opt.esdf_visual_mode      = ui.esdf_visual_mode;
            opt.esdf_color_mode       = ui.esdf_color_mode;
            opt.esdf_z_slice_enable   = ui.esdf_z_slice_enable;
            opt.esdf_z_slice_m        = ui.esdf_z_slice_m;
            opt.esdf_use_raycast      = ui.esdf_use_raycast;
            opt.esdf_ray_origin[0]    = ui.esdf_ray_origin[0];
            opt.esdf_ray_origin[1]    = ui.esdf_ray_origin[1];
            opt.esdf_ray_origin[2]    = ui.esdf_ray_origin[2];
            opt.esdf_ray_origin_auto  = ui.esdf_ray_origin_auto;
            opt.esdf_min_ray_length   = ui.esdf_min_ray_length;
            opt.esdf_max_ray_length   = ui.esdf_max_ray_length;
            return opt;
        }

        PointCloudLoadOptions optionsFromUiBase(const PointCloudUiState& ui) {
            PointCloudLoadOptions opt;
            opt.voxel_size = ui.voxel_size;
            opt.max_points = ui.max_points;
            opt.x_min      = ui.x_min;
            opt.x_max      = ui.x_max;
            opt.y_min      = ui.y_min;
            opt.y_max      = ui.y_max;
            opt.z_min      = ui.z_min;
            opt.z_max      = ui.z_max;
            opt.color_mode = static_cast<PointCloudColorMode>(std::clamp(ui.color_mode, 0, 2));
            return opt;
        }

    }  // namespace

    int EsdfVisualModeFromString(const std::string& s) {
        if (s == "surface" || s == "Surface") {
            return 1;
        }
        if (s == "distance" || s == "distance_field" || s == "Distance") {
            return 2;
        }
        return 0;
    }

    int EsdfColorModeFromString(const std::string& s) {
        if (s == "flat" || s == "Flat" || s == "gray") {
            return 0;
        }
        if (s == "distance" || s == "Distance" || s == "esdf") {
            return 2;
        }
        return 1;  // height / height_z / z
    }

    PointCloudColorMode PointCloudColorModeFromString(const std::string& s) {
        if (s == "flat" || s == "Flat") {
            return PointCloudColorMode::Flat;
        }
        if (s == "intensity" || s == "Intensity") {
            return PointCloudColorMode::Intensity;
        }
        return PointCloudColorMode::Height;
    }

    KinematicPointCloudLayer::KinematicPointCloudLayer() = default;

    KinematicPointCloudLayer::~KinematicPointCloudLayer() {
        releaseGpu();
    }

    void KinematicPointCloudLayer::releaseGpu() {
        if (vbo_color_ != 0) {
            glDeleteBuffers(1, &vbo_color_);
            vbo_color_ = 0;
        }
        if (vbo_pos_ != 0) {
            glDeleteBuffers(1, &vbo_pos_);
            vbo_pos_ = 0;
        }
        if (vao_ != 0) {
            glDeleteVertexArrays(1, &vao_);
            vao_ = 0;
        }
        gpu_point_count_ = 0;
    }

    void KinematicPointCloudLayer::Clear() {
        positions_.clear();
        colors_.clear();
        releaseGpu();
    }

    bool KinematicPointCloudLayer::LoadFromFile(const std::string& path, const PointCloudLoadOptions& options, std::string* status) {
        if (options.build_esdf) {
            return LoadEsdfFromPcd(path, options, status);
        }
        Clear();

#ifndef RKV_HAS_PCL
        if (status) {
            *status = "未编译点云插件支持";
        }
        return false;
#else
        if (!std::filesystem::exists(path)) {
            if (status) {
                *status = "文件不存在: " + path;
            }
            return false;
        }

        if (!plugin().ensureLoaded()) {
            if (status) {
                *status = plugin().last_error;
            }
            return false;
        }

        float* pos_ptr                 = nullptr;
        float* color_ptr               = nullptr;
        size_t count                   = 0;
        size_t source_cnt              = 0;
        char err[512]                  = {};
        const RkvPcdLoadParams cparams = toCParams(options);
        if (plugin().load(path.c_str(), &cparams, &pos_ptr, &color_ptr, &count, &source_cnt, err, sizeof(err)) != 0) {
            if (status) {
                *status = err[0] != '\0' ? err : "加载失败";
            }
            return false;
        }

        positions_.assign(pos_ptr, pos_ptr + count * 3);
        colors_.assign(color_ptr, color_ptr + count * 3);
        plugin().free_fn(pos_ptr);
        plugin().free_fn(color_ptr);

        uploadToGpu();

        if (status) {
            std::ostringstream oss;
            oss << "已加载 " << gpu_point_count_ << " 点 (原始 " << source_cnt << ")";
            *status = oss.str();
        }
        return gpu_point_count_ > 0;
#endif
    }

    bool KinematicPointCloudLayer::LoadEsdfFromPcd(const std::string& path, const PointCloudLoadOptions& options, std::string* status) {
        Clear();

#ifndef RKV_HAS_PCL
        if (status) {
            *status = "未编译点云插件支持";
        }
        return false;
#else
        if (!std::filesystem::exists(path)) {
            if (status) {
                *status = "文件不存在: " + path;
            }
            return false;
        }

        if (!plugin().ensureLoaded() || plugin().build_esdf == nullptr) {
            if (status) {
                *status = plugin().last_error.empty() ? "ESDF 插件符号缺失" : plugin().last_error;
            }
            return false;
        }

        float* pos_ptr                = nullptr;
        float* color_ptr              = nullptr;
        size_t count                  = 0;
        size_t source_cnt             = 0;
        char err[512]                 = {};
        const RkvPcdLoadParams crop   = toCParams(options);
        const RkvEsdfBuildParams esdf = toEsdfParams(options);
        if (plugin().build_esdf(path.c_str(), &crop, &esdf, &pos_ptr, &color_ptr, &count, &source_cnt, err, sizeof(err)) != 0) {
            if (status) {
                *status = err[0] != '\0' ? err : "ESDF 构建失败";
            }
            return false;
        }

        positions_.assign(pos_ptr, pos_ptr + count * 3);
        colors_.assign(color_ptr, color_ptr + count * 3);
        plugin().free_fn(pos_ptr);
        plugin().free_fn(color_ptr);
        uploadToGpu();

        if (status) {
            std::ostringstream oss;
            oss << "ESDF 可视化 " << gpu_point_count_ << " 点 (PCD 输入 " << source_cnt << ")";
            if (err[0] != '\0') {
                oss << " — " << err;
            }
            *status = oss.str();
        }
        return gpu_point_count_ > 0;
#endif
    }

    void KinematicPointCloudLayer::uploadToGpu() {
        releaseGpu();
        const size_t n = positions_.size() / 3;
        if (n == 0) {
            return;
        }

        glGenVertexArrays(1, &vao_);
        glGenBuffers(1, &vbo_pos_);
        glGenBuffers(1, &vbo_color_);

        glBindVertexArray(vao_);

        glBindBuffer(GL_ARRAY_BUFFER, vbo_pos_);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(positions_.size() * sizeof(float)), positions_.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

        glBindBuffer(GL_ARRAY_BUFFER, vbo_color_);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(colors_.size() * sizeof(float)), colors_.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

        glBindVertexArray(0);
        gpu_point_count_ = n;
    }

    void KinematicPointCloudLayer::Draw(GLuint shader, const glm::mat4& view, const glm::mat4& proj, const glm::mat4& model,
                                        float point_size_px) const {
        if (vao_ == 0 || gpu_point_count_ == 0) {
            return;
        }

        glUseProgram(shader);
        glUniformMatrix4fv(glGetUniformLocation(shader, "view"), 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(glGetUniformLocation(shader, "projection"), 1, GL_FALSE, glm::value_ptr(proj));
        glUniformMatrix4fv(glGetUniformLocation(shader, "model"), 1, GL_FALSE, glm::value_ptr(model));
        glUniform1f(glGetUniformLocation(shader, "pointSize"), std::max(1.0f, point_size_px));

        glEnable(GL_PROGRAM_POINT_SIZE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_TRUE);

        glBindVertexArray(vao_);
        glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(gpu_point_count_));
        glBindVertexArray(0);

        glDisable(GL_BLEND);
    }

    void RenderPointCloudPanel(PointCloudUiState* state, KinematicPointCloudLayer* layer) {
        if (state == nullptr || layer == nullptr) {
            return;
        }

#ifndef RKV_HAS_PCL
        ImGui::TextColored(ImVec4(1, 0.4f, 0.3f, 1), "当前构建未启用点云插件");
        return;
#endif

        ImGui::Checkbox("显示点云", &state->visible);
        ImGui::InputText("PCD 路径", state->file_path, sizeof(state->file_path));
        ImGui::InputFloat("体素 (m)", &state->voxel_size, 0.005f, 0.05f, "%.3f");
        ImGui::InputInt("最大点数", &state->max_points, 50000, 200000);
        ImGui::InputFloat("map offset_x (m)", &state->offset_x, 0.01f, 0.1f, "%.3f");
        ImGui::InputFloat("map offset_y (m)", &state->offset_y, 0.01f, 0.1f, "%.3f");
        ImGui::InputFloat("map offset_yaw (deg)", &state->offset_yaw, 1.0f, 5.0f, "%.1f");
        ImGui::InputFloat("点大小 (px)", &state->point_size_px, 0.5f, 1.0f, "%.1f");

        ImGui::Text("XYZ 裁剪");
        ImGui::InputFloat("x_min", &state->x_min);
        ImGui::SameLine();
        ImGui::InputFloat("x_max", &state->x_max);
        ImGui::InputFloat("y_min", &state->y_min);
        ImGui::SameLine();
        ImGui::InputFloat("y_max", &state->y_max);
        ImGui::InputFloat("z_min", &state->z_min);
        ImGui::SameLine();
        ImGui::InputFloat("z_max", &state->z_max);

        const char* color_items[] = {"单色", "高度(Z)", "强度"};
        ImGui::Combo("着色", &state->color_mode, color_items, 3);

        ImGui::Separator();
        ImGui::Checkbox("从 PCD 构建 ESDF", &state->build_esdf);
        if (state->build_esdf) {
            ImGui::InputFloat("ESDF 分辨率 (m)", &state->esdf_resolution, 0.005f, 0.02f, "%.3f");
            ImGui::Checkbox("固定地图范围", &state->esdf_use_fixed_map);
            if (state->esdf_use_fixed_map) {
                ImGui::InputFloat3("origin", state->esdf_map_origin);
                ImGui::InputFloat3("map_size", state->esdf_map_size);
            }
            const char* vis_modes[] = {"占据栅格 (RViz Map)", "障碍表面", "距离场 (稠密)"};
            ImGui::Combo("显示模式", &state->esdf_visual_mode, vis_modes, 3);
            const char* esdf_color_items[] = {"单色灰", "高度 Z (RViz)", "ESDF 距离"};
            ImGui::Combo("体素着色", &state->esdf_color_mode, esdf_color_items, 3);
            if (state->esdf_visual_mode > 0) {
                ImGui::InputFloat(state->esdf_visual_mode == 1 ? "表面厚度 (m)" : "可视距离 (m)", &state->esdf_max_visual_dist, 0.02f, 0.1f,
                                  "%.2f");
            }
            if (state->esdf_visual_mode == 2) {
                ImGui::InputInt("stride", &state->esdf_visual_stride, 1, 2);
            }
            ImGui::Checkbox("Z 切片", &state->esdf_z_slice_enable);
            if (state->esdf_z_slice_enable) {
                ImGui::InputFloat("切片高度 (m)", &state->esdf_z_slice_m, 0.05f, 0.2f, "%.2f");
            }
            ImGui::Separator();
            ImGui::Checkbox("Raycast 更新", &state->esdf_use_raycast);
            ImGui::Checkbox("传感器原点自动(地图中心)", &state->esdf_ray_origin_auto);
            if (!state->esdf_ray_origin_auto) {
                ImGui::InputFloat3("ray_origin", state->esdf_ray_origin);
            }
            ImGui::InputFloat("min_ray (m)", &state->esdf_min_ray_length, 0.05f, 0.5f, "%.2f");
            ImGui::InputFloat("max_ray (m)", &state->esdf_max_ray_length, 1.0f, 5.0f, "%.1f");
            ImGui::TextDisabled("关闭 Raycast = 仅打点占据 (load_ESDFMap PCD 路径)");
            ImGui::TextDisabled("栅格模式: 调大「点大小」≈ 体素边长，更像 RViz");
        }

        if (ImGui::Button("加载 / 重新加载")) {
            PointCloudLoadOptions opt = optionsFromUi(*state);
            opt.color_mode            = static_cast<PointCloudColorMode>(state->color_mode);
            std::string load_status;
            if (layer->LoadFromFile(state->file_path, opt, &load_status)) {
                state->loaded          = true;
                state->rendered_points = layer->gpuPointCount();
                std::snprintf(state->last_status, sizeof(state->last_status), "%s", load_status.c_str());
            } else {
                state->loaded = false;
                std::snprintf(state->last_status, sizeof(state->last_status), "%s", load_status.c_str());
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("清除")) {
            layer->Clear();
            state->loaded          = false;
            state->rendered_points = 0;
            std::snprintf(state->last_status, sizeof(state->last_status), "已清除");
        }

        if (state->last_status[0] != '\0') {
            ImGui::TextWrapped("%s", state->last_status);
        }
        if (state->loaded) {
            ImGui::Text("GPU 点数: %zu", state->rendered_points);
        }
    }

}  // namespace kinematic_viewer
