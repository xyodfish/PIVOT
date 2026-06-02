#include "kinematic_viewer/rkv_esdf_grid.h"
#include "kinematic_viewer/rkv_raycast.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace kinematic_viewer {
    namespace {

        constexpr double kInfinityDist = 10000.0;

        struct Rgb {
            float r, g, b;
        };

        Rgb rainbow(float t) {
            t = std::max(0.0f, std::min(1.0f, t));
            if (t < 0.5f) {
                const float u = t * 2.0f;
                return {0.0f, u, 1.0f - u};
            }
            const float u = (t - 0.5f) * 2.0f;
            return {u, 1.0f - u, 0.0f};
        }

        Rgb heightColor(float t) {
            t = std::max(0.0f, std::min(1.0f, t));
            if (t < 0.5f) {
                const float u = t * 2.0f;
                return {0.0f, u, 1.0f - u};
            }
            const float u = (t - 0.5f) * 2.0f;
            return {u, 1.0f - u, 0.0f};
        }

    }  // namespace

    bool RkvEsdfGrid::distanceValid(double d) const {
        return d >= 0.0 && d < kInfinityDist * 0.5;
    }

    int RkvEsdfGrid::index(int x, int y, int z) const {
        return z * nxy_ + y * nx_ + x;
    }

    void RkvEsdfGrid::posToVox(double x, double y, double z, int* vx, int* vy, int* vz) const {
        *vx = static_cast<int>(std::floor((x - origin_[0]) / resolution_));
        *vy = static_cast<int>(std::floor((y - origin_[1]) / resolution_));
        *vz = static_cast<int>(std::floor((z - origin_[2]) / resolution_));
    }

    void RkvEsdfGrid::voxToPos(int vx, int vy, int vz, float* px, float* py, float* pz) const {
        *px = static_cast<float>((vx + 0.5) * resolution_ + origin_[0]);
        *py = static_cast<float>((vy + 0.5) * resolution_ + origin_[1]);
        *pz = static_cast<float>((vz + 0.5) * resolution_ + origin_[2]);
    }

    bool RkvEsdfGrid::initGrid(const RkvEsdfGridParams& params, const double min_b[3], const double max_b[3], std::string* err) {
        resolution_ = params.resolution;

        const bool use_fixed_map = !params.auto_bounds && params.map_size[0] > 0.0 && params.map_size[1] > 0.0 && params.map_size[2] > 0.0;

        if (use_fixed_map) {
            origin_[0] = params.origin[0];
            origin_[1] = params.origin[1];
            origin_[2] = params.origin[2];
            nx_        = static_cast<int>(std::ceil(params.map_size[0] / resolution_));
            ny_        = static_cast<int>(std::ceil(params.map_size[1] / resolution_));
            nz_        = static_cast<int>(std::ceil(params.map_size[2] / resolution_));
        } else {
            const double pad = params.padding;
            origin_[0]       = min_b[0] - pad;
            origin_[1]       = min_b[1] - pad;
            origin_[2]       = min_b[2] - pad;
            nx_              = static_cast<int>(std::ceil((max_b[0] - min_b[0] + 2.0 * pad) / resolution_));
            ny_              = static_cast<int>(std::ceil((max_b[1] - min_b[1] + 2.0 * pad) / resolution_));
            nz_              = static_cast<int>(std::ceil((max_b[2] - min_b[2] + 2.0 * pad) / resolution_));
        }

        if (nx_ <= 0 || ny_ <= 0 || nz_ <= 0) {
            if (err) {
                *err = "invalid esdf grid dimensions";
            }
            return false;
        }

        grid_total_size_ = nx_ * ny_ * nz_;
        nxy_             = nx_ * ny_;
        if (static_cast<size_t>(grid_total_size_) > params.max_voxels) {
            if (err) {
                std::ostringstream oss;
                oss << "esdf grid too large (" << nx_ << "x" << ny_ << "x" << nz_ << "=" << grid_total_size_
                    << " voxels); enlarge esdf_resolution (coarser voxels) or shrink map_size";
                *err = oss.str();
            }
            return false;
        }

        reserved_idx_undefined_ = grid_total_size_;
        resetFiestaBuffers();
        free_stamp_.assign(static_cast<size_t>(grid_total_size_), 0);
        occ_stamp_.assign(static_cast<size_t>(grid_total_size_), 0);
        return true;
    }

    int RkvEsdfGrid::setOccupancyWorld(double x, double y, double z, uint8_t occ) {
        fiesta::Vox3i vox;
        posToVox(x, y, z, &vox.x, &vox.y, &vox.z);
        return setOccupancyVox(vox, occ);
    }

    void RkvEsdfGrid::markOccupiedDirect(const std::vector<float>& xyz, size_t point_count) {
        for (size_t i = 0; i < point_count; ++i) {
            setOccupancyWorld(xyz[i * 3 + 0], xyz[i * 3 + 1], xyz[i * 3 + 2], 1);
        }
    }

    void RkvEsdfGrid::raycastOnePoint(double px, double py, double pz, const RkvEsdfGridParams& params, int stamp) {
        const double half = 0.5;
        double ray_ox     = params.ray_origin[0];
        double ray_oy     = params.ray_origin[1];
        double ray_oz     = params.ray_origin[2];
        if (params.ray_origin_auto) {
            ray_ox = origin_[0] + 0.5 * nx_ * resolution_;
            ray_oy = origin_[1] + 0.5 * ny_ * resolution_;
            ray_oz = origin_[2];
        }

        double point_x = px;
        double point_y = py;
        double point_z = pz;

        double length = std::sqrt((point_x - ray_ox) * (point_x - ray_ox) + (point_y - ray_oy) * (point_y - ray_oy) +
                                  (point_z - ray_oz) * (point_z - ray_oz));
        if (length < params.min_ray_length) {
            return;
        }

        int tmp_idx = -1;
        if (length > params.max_ray_length) {
            const double scale = params.max_ray_length / length;
            point_x            = (point_x - ray_ox) * scale + ray_ox;
            point_y            = (point_y - ray_oy) * scale + ray_oy;
            point_z            = (point_z - ray_oz) * scale + ray_oz;
            tmp_idx            = setOccupancyWorld(point_x, point_y, point_z, 0);
        } else {
            tmp_idx = setOccupancyWorld(point_x, point_y, point_z, 1);
        }

        if (tmp_idx >= 0) {
            if (occ_stamp_[static_cast<size_t>(tmp_idx)] == stamp) {
                return;
            }
            occ_stamp_[static_cast<size_t>(tmp_idx)] = stamp;
        }

        const RkvVec3d start_vox{(ray_ox - origin_[0]) / resolution_, (ray_oy - origin_[1]) / resolution_,
                                 (ray_oz - origin_[2]) / resolution_};
        const RkvVec3d end_vox{(point_x - origin_[0]) / resolution_, (point_y - origin_[1]) / resolution_,
                               (point_z - origin_[2]) / resolution_};
        const RkvVec3d grid_min{0.0, 0.0, 0.0};
        const RkvVec3d grid_max{static_cast<double>(nx_), static_cast<double>(ny_), static_cast<double>(nz_)};

        ray_voxels_scratch_.clear();
        RkvRaycast(start_vox, end_vox, grid_min, grid_max, &ray_voxels_scratch_);

        int free_repeat_cnt = 0;
        for (int i = static_cast<int>(ray_voxels_scratch_.size()) - 2; i >= 0; --i) {
            const double wx = (ray_voxels_scratch_[static_cast<size_t>(i)].x + half) * resolution_ + origin_[0];
            const double wy = (ray_voxels_scratch_[static_cast<size_t>(i)].y + half) * resolution_ + origin_[1];
            const double wz = (ray_voxels_scratch_[static_cast<size_t>(i)].z + half) * resolution_ + origin_[2];

            length = std::sqrt((wx - ray_ox) * (wx - ray_ox) + (wy - ray_oy) * (wy - ray_oy) + (wz - ray_oz) * (wz - ray_oz));
            if (length < params.min_ray_length) {
                break;
            }
            if (length > params.max_ray_length) {
                continue;
            }

            tmp_idx = setOccupancyWorld(wx, wy, wz, 0);
            if (tmp_idx < 0) {
                continue;
            }

            if (free_stamp_[static_cast<size_t>(tmp_idx)] == stamp) {
                if (++free_repeat_cnt >= 1) {
                    free_repeat_cnt = 0;
                    break;
                }
            } else {
                free_stamp_[static_cast<size_t>(tmp_idx)] = stamp;
                free_repeat_cnt                           = 0;
            }
        }
    }

    void RkvEsdfGrid::integrateRaycast(const std::vector<float>& xyz, size_t point_count, const RkvEsdfGridParams& params) {
        const int stamp = 1;
        for (size_t i = 0; i < point_count; ++i) {
            const float px = xyz[i * 3 + 0];
            const float py = xyz[i * 3 + 1];
            const float pz = xyz[i * 3 + 2];
            if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) {
                continue;
            }
            raycastOnePoint(px, py, pz, params, stamp);
        }
    }

    bool RkvEsdfGrid::buildFromPoints(const std::vector<float>& xyz, size_t point_count, const RkvEsdfGridParams& params,
                                      std::string* err) {
        if (point_count == 0 || xyz.size() < point_count * 3) {
            if (err) {
                *err = "empty point cloud";
            }
            return false;
        }
        if (params.resolution <= 1e-6) {
            if (err) {
                *err = "invalid esdf resolution";
            }
            return false;
        }

        double min_b[3] = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
        double max_b[3] = {std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(),
                           std::numeric_limits<double>::lowest()};

        for (size_t i = 0; i < point_count; ++i) {
            min_b[0] = std::min(min_b[0], static_cast<double>(xyz[i * 3 + 0]));
            min_b[1] = std::min(min_b[1], static_cast<double>(xyz[i * 3 + 1]));
            min_b[2] = std::min(min_b[2], static_cast<double>(xyz[i * 3 + 2]));
            max_b[0] = std::max(max_b[0], static_cast<double>(xyz[i * 3 + 0]));
            max_b[1] = std::max(max_b[1], static_cast<double>(xyz[i * 3 + 1]));
            max_b[2] = std::max(max_b[2], static_cast<double>(xyz[i * 3 + 2]));
        }

        if (!initGrid(params, min_b, max_b, err)) {
            return false;
        }

        if (params.use_raycast) {
            integrateRaycast(xyz, point_count, params);
        } else {
            markOccupiedDirect(xyz, point_count);
        }

        if (params.compute_distance_field) {
            updateEsdfFiesta();
        }
        return true;
    }

    bool RkvEsdfGrid::sampleVisualization(RkvEsdfVisualMode mode, RkvEsdfColorMode color_mode, float max_dist_m, int stride, int max_points,
                                          bool z_slice_enable, float z_slice_m, std::vector<float>* positions, std::vector<float>* colors,
                                          size_t* occupied_voxels, std::string* err) const {
        if (positions == nullptr || colors == nullptr) {
            if (err) {
                *err = "null output";
            }
            return false;
        }

        stride     = std::max(1, stride);
        max_points = std::max(1000, max_points);

        positions->clear();
        colors->clear();
        size_t occ_count = 0;

        const float slice_half = static_cast<float>(resolution_ * 0.55);
        const Rgb occ_color{0.72f, 0.74f, 0.78f};
        const float z_lo   = static_cast<float>(origin_[2]);
        const float z_hi   = static_cast<float>(origin_[2] + nz_ * resolution_);
        const float z_span = std::max(z_hi - z_lo, 1e-3f);

        auto pickColor = [&](float px, float py, float pz, float dist_m) -> Rgb {
            switch (color_mode) {
                case RkvEsdfColorMode::HeightZ:
                    return heightColor((pz - z_lo) / z_span);
                case RkvEsdfColorMode::Distance: {
                    const float t = max_dist_m > 1e-6f ? std::min(1.0f, dist_m / max_dist_m) : std::min(1.0f, dist_m / 2.0f);
                    return rainbow(t);
                }
                default:
                    (void)px;
                    (void)py;
                    (void)pz;
                    (void)dist_m;
                    return occ_color;
            }
        };

        for (int z = 0; z < nz_; z += stride) {
            for (int y = 0; y < ny_; y += stride) {
                for (int x = 0; x < nx_; x += stride) {
                    const int idx = index(x, y, z);
                    if (occupancy_buffer_[static_cast<size_t>(idx)] != 0) {
                        ++occ_count;
                    }

                    float px = 0.0f;
                    float py = 0.0f;
                    float pz = 0.0f;
                    voxToPos(x, y, z, &px, &py, &pz);
                    if (z_slice_enable && std::fabs(pz - z_slice_m) > slice_half) {
                        continue;
                    }

                    const double dist_m = distance_buffer_[static_cast<size_t>(idx)];
                    bool draw           = false;
                    switch (mode) {
                        case RkvEsdfVisualMode::Occupied:
                            draw = occupancy_buffer_[static_cast<size_t>(idx)] == 1;
                            break;
                        case RkvEsdfVisualMode::Surface:
                            if (!distanceValid(dist_m)) {
                                break;
                            }
                            draw = static_cast<float>(dist_m) <= max_dist_m;
                            break;
                        case RkvEsdfVisualMode::DistanceField:
                            if (!distanceValid(dist_m) || static_cast<float>(dist_m) > max_dist_m) {
                                break;
                            }
                            draw = true;
                            break;
                    }

                    if (!draw) {
                        continue;
                    }

                    const Rgb c = pickColor(px, py, pz, static_cast<float>(dist_m));
                    positions->push_back(px);
                    positions->push_back(py);
                    positions->push_back(pz);
                    colors->push_back(c.r);
                    colors->push_back(c.g);
                    colors->push_back(c.b);
                    if (static_cast<int>(positions->size() / 3) >= max_points) {
                        if (occupied_voxels != nullptr) {
                            *occupied_voxels = occ_count;
                        }
                        return true;
                    }
                }
            }
        }

        if (positions->empty()) {
            if (err) {
                *err = "no esdf voxels to display";
            }
            return false;
        }
        if (occupied_voxels != nullptr) {
            *occupied_voxels = occ_count;
        }
        return true;
    }

}  // namespace kinematic_viewer
