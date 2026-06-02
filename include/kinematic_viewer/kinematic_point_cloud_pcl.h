#pragma once

#include <cstddef>

#ifdef __cplusplus
extern "C" {
#endif

struct RkvPcdLoadParams {
    float voxel_size;
    int max_points;
    float x_min;
    float x_max;
    float y_min;
    float y_max;
    float z_min;
    float z_max;
    int color_mode;  // 0 flat, 1 height, 2 intensity
};

// Loads and filters a PCD file. On success returns 0 and sets out_* (caller must rkv_pcd_free).
int rkv_pcd_load_file(const char* path, const RkvPcdLoadParams* params, float** out_positions, float** out_colors, size_t* out_point_count,
                      size_t* out_source_count, char* err_buf, size_t err_buf_size);

void rkv_pcd_free(float* buffer);

struct RkvEsdfBuildParams {
    float esdf_resolution;
    float map_origin[3];
    float map_size[3];
    int use_fixed_map;  // 1 => map_origin + map_size from config (aphropm-style)
    float padding;
    float x_min;
    float x_max;
    float y_min;
    float y_max;
    float z_min;
    float z_max;
    float max_visual_dist;
    int visual_stride;
    int max_points;
    int visual_mode;  // 0 occupied  1 surface  2 distance field
    int color_mode;   // 0 flat  1 height  2 distance
    int z_slice_enable;
    float z_slice_m;
    int use_raycast;
    float ray_origin[3];
    int ray_origin_auto;
    float min_ray_length;
    float max_ray_length;
};

// Build ESDF from PCD (occupancy + EDT), export colored voxel samples for GL.
int rkv_esdf_build_from_pcd(const char* path, const RkvPcdLoadParams* crop, const RkvEsdfBuildParams* esdf, float** out_positions,
                            float** out_colors, size_t* out_point_count, size_t* out_source_count, char* err_buf, size_t err_buf_size);

#ifdef __cplusplus
}
#endif
