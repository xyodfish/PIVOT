#include "kinematic_viewer/rkv_raycast.h"

#include <cmath>
#include <iostream>

namespace kinematic_viewer {
    namespace {

        int SignumInt(int x) {
            return x == 0 ? 0 : (x < 0 ? -1 : 1);
        }

        double Mod1(double value, double modulus) {
            return std::fmod(std::fmod(value, modulus) + modulus, modulus);
        }

        // aphropm raycast.cpp intbound
        double IntBound(double s, double ds) {
            if (ds < 0) {
                return IntBound(-s, -ds);
            }
            s = Mod1(s, 1.0);
            return (1.0 - s) / ds;
        }

    }  // namespace

    void RkvRaycast(const RkvVec3d& start, const RkvVec3d& end, const RkvVec3d& grid_min, const RkvVec3d& grid_max,
                    std::vector<RkvVec3d>* output) {
        if (output == nullptr) {
            return;
        }

        int x           = static_cast<int>(std::floor(start.x));
        int y           = static_cast<int>(std::floor(start.y));
        int z           = static_cast<int>(std::floor(start.z));
        const int end_x = static_cast<int>(std::floor(end.x));
        const int end_y = static_cast<int>(std::floor(end.y));
        const int end_z = static_cast<int>(std::floor(end.z));

        const double dx = static_cast<double>(end_x - x);
        const double dy = static_cast<double>(end_y - y);
        const double dz = static_cast<double>(end_z - z);

        const int step_x = SignumInt(static_cast<int>(dx));
        const int step_y = SignumInt(static_cast<int>(dy));
        const int step_z = SignumInt(static_cast<int>(dz));

        double t_max_x = IntBound(start.x, dx);
        double t_max_y = IntBound(start.y, dy);
        double t_max_z = IntBound(start.z, dz);

        const double t_delta_x = static_cast<double>(step_x) / dx;
        const double t_delta_y = static_cast<double>(step_y) / dy;
        const double t_delta_z = static_cast<double>(step_z) / dz;

        const double dir_x       = end.x - start.x;
        const double dir_y       = end.y - start.y;
        const double dir_z       = end.z - start.z;
        const double max_dist_sq = dir_x * dir_x + dir_y * dir_y + dir_z * dir_z;

        output->clear();

        if (step_x == 0 && step_y == 0 && step_z == 0) {
            return;
        }

        while (true) {
            if (x >= static_cast<int>(grid_min.x) && x < static_cast<int>(grid_max.x) && y >= static_cast<int>(grid_min.y) &&
                y < static_cast<int>(grid_max.y) && z >= static_cast<int>(grid_min.z) && z < static_cast<int>(grid_max.z)) {
                const double vx      = static_cast<double>(x) - start.x;
                const double vy      = static_cast<double>(y) - start.y;
                const double vz      = static_cast<double>(z) - start.z;
                const double dist_sq = vx * vx + vy * vy + vz * vz;
                output->push_back({static_cast<double>(x), static_cast<double>(y), static_cast<double>(z)});
                if (dist_sq > max_dist_sq) {
                    return;
                }
                if (output->size() > 1500) {
                    std::cerr << "[rkv_raycast] too many voxels along ray\n";
                    return;
                }
            }

            if (x == end_x && y == end_y && z == end_z) {
                break;
            }

            if (t_max_x < t_max_y) {
                if (t_max_x < t_max_z) {
                    x += step_x;
                    t_max_x += t_delta_x;
                } else {
                    z += step_z;
                    t_max_z += t_delta_z;
                }
            } else {
                if (t_max_y < t_max_z) {
                    y += step_y;
                    t_max_y += t_delta_y;
                } else {
                    z += step_z;
                    t_max_z += t_delta_z;
                }
            }
        }
    }

}  // namespace kinematic_viewer
