#include "kinematic_viewer/kinematic_point_cloud_pcl.h"
#include "kinematic_viewer/rkv_esdf_grid.h"

#include <pcl/common/io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>

namespace {

    void setError(char* err_buf, size_t err_buf_size, const std::string& msg) {
        if (err_buf == nullptr || err_buf_size == 0) {
            return;
        }
        std::snprintf(err_buf, err_buf_size, "%s", msg.c_str());
    }

    struct Rgb {
        float r, g, b;
    };

    Rgb heightColor(float t) {
        t = std::max(0.0f, std::min(1.0f, t));
        if (t < 0.5f) {
            const float u = t * 2.0f;
            return {0.0f, u, 1.0f - u};
        }
        const float u = (t - 0.5f) * 2.0f;
        return {u, 1.0f - u, 0.0f};
    }

    bool loadXyzCloud(const char* path, pcl::PointCloud<pcl::PointXYZ>::Ptr* out, std::string* err) {
        pcl::PCLPointCloud2 blob;
        if (pcl::io::loadPCDFile(path, blob) != 0) {
            if (err) {
                *err = std::string("loadPCDFile failed: ") + path;
            }
            return false;
        }

        pcl::PointCloud<pcl::PointXYZ> cloud;
        try {
            pcl::fromPCLPointCloud2(blob, cloud);
        } catch (const std::exception& e) {
            if (err) {
                *err = std::string("fromPCLPointCloud2: ") + e.what();
            }
            return false;
        }

        *out = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(std::move(cloud)));
        return !(*out)->empty();
    }

    bool filterCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input, const RkvPcdLoadParams& opt,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr* out, size_t* source_count) {
        *source_count = input->size();
        if (input->empty()) {
            return false;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered = input;

        auto pass = [&](const char* field, float lo, float hi) {
            pcl::PassThrough<pcl::PointXYZ> pass_filter;
            pass_filter.setInputCloud(filtered);
            pass_filter.setFilterFieldName(field);
            pass_filter.setFilterLimits(lo, hi);
            pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
            pass_filter.filter(*tmp);
            filtered.swap(tmp);
        };

        pass("x", opt.x_min, opt.x_max);
        pass("y", opt.y_min, opt.y_max);
        pass("z", opt.z_min, opt.z_max);

        if (opt.voxel_size > 1e-6f) {
            pcl::VoxelGrid<pcl::PointXYZ> voxel;
            voxel.setInputCloud(filtered);
            voxel.setLeafSize(opt.voxel_size, opt.voxel_size, opt.voxel_size);
            pcl::PointCloud<pcl::PointXYZ>::Ptr down(new pcl::PointCloud<pcl::PointXYZ>);
            voxel.filter(*down);
            filtered = down;
        }

        if (opt.max_points > 0 && static_cast<int>(filtered->size()) > opt.max_points) {
            filtered->points.resize(static_cast<size_t>(opt.max_points));
            filtered->width  = static_cast<uint32_t>(filtered->size());
            filtered->height = 1;
        }

        *out = filtered;
        return !filtered->empty();
    }

    bool fillBuffers(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const RkvPcdLoadParams& opt, float** out_positions,
                     float** out_colors, size_t* out_point_count) {
        const size_t n = cloud->size();
        if (n == 0) {
            return false;
        }

        float* positions = static_cast<float*>(std::malloc(n * 3 * sizeof(float)));
        float* colors    = static_cast<float*>(std::malloc(n * 3 * sizeof(float)));
        if (positions == nullptr || colors == nullptr) {
            std::free(positions);
            std::free(colors);
            return false;
        }

        float z_min = std::numeric_limits<float>::max();
        float z_max = std::numeric_limits<float>::lowest();
        for (size_t i = 0; i < n; ++i) {
            z_min = std::min(z_min, cloud->points[i].z);
            z_max = std::max(z_max, cloud->points[i].z);
        }
        if (z_max <= z_min) {
            z_max = z_min + 1e-3f;
        }

        const Rgb flat{0.55f, 0.58f, 0.62f};
        for (size_t i = 0; i < n; ++i) {
            const auto& p        = cloud->points[i];
            positions[i * 3 + 0] = p.x;
            positions[i * 3 + 1] = p.y;
            positions[i * 3 + 2] = p.z;

            Rgb c = flat;
            if (opt.color_mode == 1) {
                c = heightColor((p.z - z_min) / (z_max - z_min));
            }
            colors[i * 3 + 0] = c.r;
            colors[i * 3 + 1] = c.g;
            colors[i * 3 + 2] = c.b;
        }

        *out_positions   = positions;
        *out_colors      = colors;
        *out_point_count = n;
        return true;
    }

}  // namespace

extern "C" int rkv_pcd_load_file(const char* path, const RkvPcdLoadParams* params, float** out_positions, float** out_colors,
                                 size_t* out_point_count, size_t* out_source_count, char* err_buf, size_t err_buf_size) {
    if (path == nullptr || params == nullptr || out_positions == nullptr || out_colors == nullptr || out_point_count == nullptr) {
        setError(err_buf, err_buf_size, "invalid arguments");
        return -1;
    }

    *out_positions   = nullptr;
    *out_colors      = nullptr;
    *out_point_count = 0;
    if (out_source_count != nullptr) {
        *out_source_count = 0;
    }

    std::string err;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    if (!loadXyzCloud(path, &cloud, &err)) {
        setError(err_buf, err_buf_size, err);
        return -1;
    }

    size_t source_count = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered;
    if (!filterCloud(cloud, *params, &filtered, &source_count)) {
        setError(err_buf, err_buf_size, "no points after filter");
        return -1;
    }
    if (out_source_count != nullptr) {
        *out_source_count = source_count;
    }

    if (!fillBuffers(filtered, *params, out_positions, out_colors, out_point_count)) {
        setError(err_buf, err_buf_size, "out of memory");
        return -1;
    }

    return 0;
}

extern "C" void rkv_pcd_free(float* buffer) {
    std::free(buffer);
}

extern "C" int rkv_esdf_build_from_pcd(const char* path, const RkvPcdLoadParams* crop, const RkvEsdfBuildParams* esdf,
                                       float** out_positions, float** out_colors, size_t* out_point_count, size_t* out_source_count,
                                       char* err_buf, size_t err_buf_size) {
    if (path == nullptr || crop == nullptr || esdf == nullptr || out_positions == nullptr || out_colors == nullptr ||
        out_point_count == nullptr) {
        setError(err_buf, err_buf_size, "invalid arguments");
        return -1;
    }

    *out_positions   = nullptr;
    *out_colors      = nullptr;
    *out_point_count = 0;
    if (out_source_count != nullptr) {
        *out_source_count = 0;
    }

    const auto t0 = std::chrono::steady_clock::now();
    auto logMs    = [&](const char* phase) {
        const auto now = std::chrono::steady_clock::now();
        const auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(now - t0).count();
        std::fprintf(stderr, "[rkv_esdf] %s +%lld ms\n", phase, static_cast<long long>(ms));
    };

    std::string err;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    if (!loadXyzCloud(path, &cloud, &err)) {
        setError(err_buf, err_buf_size, err);
        return -1;
    }

    RkvPcdLoadParams crop_for_esdf = *crop;
    crop_for_esdf.voxel_size       = 0.0f;
    crop_for_esdf.max_points       = 0;

    size_t source_count = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered;
    if (!filterCloud(cloud, crop_for_esdf, &filtered, &source_count)) {
        setError(err_buf, err_buf_size, "no points after filter");
        return -1;
    }
    if (out_source_count != nullptr) {
        *out_source_count = source_count;
    }
    logMs("pcd_load_filter");

    std::vector<float> xyz;
    xyz.reserve(filtered->size() * 3);
    for (const auto& p : filtered->points) {
        xyz.push_back(p.x);
        xyz.push_back(p.y);
        xyz.push_back(p.z);
    }

    kinematic_viewer::RkvEsdfGridParams grid_params;
    grid_params.resolution      = esdf->esdf_resolution;
    grid_params.origin[0]       = esdf->map_origin[0];
    grid_params.origin[1]       = esdf->map_origin[1];
    grid_params.origin[2]       = esdf->map_origin[2];
    grid_params.map_size[0]     = esdf->map_size[0];
    grid_params.map_size[1]     = esdf->map_size[1];
    grid_params.map_size[2]     = esdf->map_size[2];
    grid_params.padding         = esdf->padding;
    grid_params.auto_bounds     = esdf->use_fixed_map == 0;
    grid_params.use_raycast     = esdf->use_raycast != 0;
    grid_params.ray_origin[0]   = esdf->ray_origin[0];
    grid_params.ray_origin[1]   = esdf->ray_origin[1];
    grid_params.ray_origin[2]   = esdf->ray_origin[2];
    grid_params.ray_origin_auto = esdf->ray_origin_auto != 0;
    grid_params.min_ray_length  = esdf->min_ray_length;
    grid_params.max_ray_length  = esdf->max_ray_length;

    const auto vis_mode                = static_cast<kinematic_viewer::RkvEsdfVisualMode>(std::clamp(esdf->visual_mode, 0, 2));
    grid_params.compute_distance_field = vis_mode != kinematic_viewer::RkvEsdfVisualMode::Occupied;

    kinematic_viewer::RkvEsdfGrid grid;
    if (!grid.buildFromPoints(xyz, filtered->size(), grid_params, &err)) {
        setError(err_buf, err_buf_size, err);
        return -1;
    }
    logMs(grid_params.compute_distance_field ? "grid_occ_raycast_fiesta" : "grid_occ_raycast_only");

    std::vector<float> positions;
    std::vector<float> colors;
    size_t occ_voxels   = 0;
    const auto col_mode = static_cast<kinematic_viewer::RkvEsdfColorMode>(std::clamp(esdf->color_mode, 0, 2));
    if (!grid.sampleVisualization(vis_mode, col_mode, esdf->max_visual_dist, esdf->visual_stride, esdf->max_points,
                                  esdf->z_slice_enable != 0, esdf->z_slice_m, &positions, &colors, &occ_voxels, &err)) {
        setError(err_buf, err_buf_size, err);
        return -1;
    }
    logMs("sample_visualization");

    const size_t n = positions.size() / 3;
    float* pos_ptr = static_cast<float*>(std::malloc(n * 3 * sizeof(float)));
    float* col_ptr = static_cast<float*>(std::malloc(n * 3 * sizeof(float)));
    if (pos_ptr == nullptr || col_ptr == nullptr) {
        std::free(pos_ptr);
        std::free(col_ptr);
        setError(err_buf, err_buf_size, "out of memory");
        return -1;
    }
    std::memcpy(pos_ptr, positions.data(), n * 3 * sizeof(float));
    std::memcpy(col_ptr, colors.data(), n * 3 * sizeof(float));
    *out_positions   = pos_ptr;
    *out_colors      = col_ptr;
    *out_point_count = n;

    if (err_buf != nullptr && err_buf_size > 0) {
        std::snprintf(err_buf, err_buf_size, "ESDF %dx%dx%d res=%.3f occ=%zu vis=%zu", grid.nx(), grid.ny(), grid.nz(), grid.resolution(),
                      occ_voxels, n);
    }
    return 0;
}
