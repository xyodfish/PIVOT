#pragma once

#include <glad/glad.h>
#include <glm/mat4x4.hpp>

#include <cstddef>
#include <string>
#include <vector>
#include <vector>

namespace kinematic_viewer {

    enum class PointCloudColorMode { Flat, Height, Intensity };

    struct PointCloudLoadOptions {
        float voxel_size               = 0.05f;
        int max_points                 = 500000;
        float x_min                    = -1e9f;
        float x_max                    = 1e9f;
        float y_min                    = -1e9f;
        float y_max                    = 1e9f;
        float z_min                    = -1e9f;
        float z_max                    = 1e9f;
        PointCloudColorMode color_mode = PointCloudColorMode::Height;
        bool build_esdf                = false;
        float esdf_resolution          = 0.05f;
        float esdf_map_origin[3]       = {-30.0f, -20.0f, -0.1f};
        float esdf_map_size[3]         = {60.0f, 40.0f, 2.1f};
        bool esdf_use_fixed_map        = true;
        float esdf_max_visual_dist     = 0.15f;
        int esdf_visual_stride         = 1;
        int esdf_visual_mode           = 0;  // 0 occupied 1 surface 2 distance
        int esdf_color_mode            = 1;  // 0 flat 1 height 2 distance
        bool esdf_z_slice_enable       = false;
        float esdf_z_slice_m           = 0.5f;
        bool esdf_use_raycast          = true;
        float esdf_ray_origin[3]       = {0.0f, 0.0f, 0.0f};
        bool esdf_ray_origin_auto      = true;
        float esdf_min_ray_length      = 0.1f;
        float esdf_max_ray_length      = 50.0f;
    };

    struct PointCloudUiState {
        bool visible               = true;
        bool loaded                = false;
        char file_path[512]        = "";
        char last_status[256]      = "";
        size_t source_points       = 0;
        size_t rendered_points     = 0;
        float voxel_size           = 0.05f;
        int max_points             = 500000;
        float x_min                = -100.0f;
        float x_max                = 100.0f;
        float y_min                = -100.0f;
        float y_max                = 100.0f;
        float z_min                = -1.0f;
        float z_max                = 2.0f;
        float offset_x             = 0.0f;
        float offset_y             = 0.0f;
        float offset_yaw           = 0.0f;
        float point_size_px        = 2.0f;
        int color_mode             = 1;  // 0 flat 1 height 2 intensity
        bool build_esdf            = false;
        float esdf_resolution      = 0.05f;
        float esdf_map_origin[3]   = {-30.0f, -20.0f, -0.1f};
        float esdf_map_size[3]     = {60.0f, 40.0f, 2.1f};
        bool esdf_use_fixed_map    = true;
        float esdf_max_visual_dist = 0.15f;
        int esdf_visual_stride     = 1;
        int esdf_visual_mode       = 0;
        int esdf_color_mode        = 1;
        bool esdf_z_slice_enable   = false;
        float esdf_z_slice_m       = 0.5f;
        bool esdf_use_raycast      = true;
        float esdf_ray_origin[3]   = {0.0f, 0.0f, 0.0f};
        bool esdf_ray_origin_auto  = true;
        float esdf_min_ray_length  = 0.1f;
        float esdf_max_ray_length  = 50.0f;
        bool has_intensity         = false;
        float z_color_min          = 0.0f;
        float z_color_max          = 1.0f;
    };

    PointCloudColorMode PointCloudColorModeFromString(const std::string& s);
    int EsdfVisualModeFromString(const std::string& s);
    int EsdfColorModeFromString(const std::string& s);

    class KinematicPointCloudLayer {
       public:
        KinematicPointCloudLayer();
        ~KinematicPointCloudLayer();

        KinematicPointCloudLayer(const KinematicPointCloudLayer&)            = delete;
        KinematicPointCloudLayer& operator=(const KinematicPointCloudLayer&) = delete;

        bool LoadFromFile(const std::string& path, const PointCloudLoadOptions& options, std::string* status);
        bool LoadEsdfFromPcd(const std::string& path, const PointCloudLoadOptions& options, std::string* status);
        void Clear();

        void Draw(GLuint shader, const glm::mat4& view, const glm::mat4& proj, const glm::mat4& model, float point_size_px) const;

        bool hasGpuData() const { return gpu_point_count_ > 0; }
        size_t gpuPointCount() const { return gpu_point_count_; }

       private:
        void uploadToGpu();
        void releaseGpu();

        std::vector<float> positions_;
        std::vector<float> colors_;
        size_t gpu_point_count_ = 0;
        GLuint vao_             = 0;
        GLuint vbo_pos_         = 0;
        GLuint vbo_color_       = 0;
    };

    void RenderPointCloudPanel(PointCloudUiState* state, KinematicPointCloudLayer* layer);

}  // namespace kinematic_viewer
