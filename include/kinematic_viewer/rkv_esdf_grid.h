#pragma once

#include "kinematic_viewer/rkv_fiesta_dirs.h"
#include "kinematic_viewer/rkv_raycast.h"

#include <cstddef>
#include <cstdint>
#include <queue>
#include <string>
#include <vector>

namespace kinematic_viewer {

    enum class RkvEsdfVisualMode { Occupied = 0, Surface = 1, DistanceField = 2 };
    enum class RkvEsdfColorMode { Flat = 0, HeightZ = 1, Distance = 2 };

    struct RkvEsdfGridParams {
        double resolution     = 0.05;
        double origin[3]      = {0.0, 0.0, 0.0};
        double map_size[3]    = {0.0, 0.0, 0.0};
        double padding        = 0.05;
        bool auto_bounds      = true;
        size_t max_voxels     = 67108864;
        bool use_raycast      = true;
        double ray_origin[3]  = {0.0, 0.0, 0.0};
        bool ray_origin_auto  = true;
        double min_ray_length = 0.1;
        double max_ray_length = 50.0;
        // false when only visualizing occupancy (skips FIESTA UpdateESDF).
        bool compute_distance_field = true;
    };

    // Dense occupancy + FIESTA ESDF (aphropm ESDFMap, non-PROBABILISTIC).
    class RkvEsdfGrid {
       public:
        bool buildFromPoints(const std::vector<float>& xyz, size_t point_count, const RkvEsdfGridParams& params, std::string* err);

        bool sampleVisualization(RkvEsdfVisualMode mode, RkvEsdfColorMode color_mode, float max_dist_m, int stride, int max_points,
                                 bool z_slice_enable, float z_slice_m, std::vector<float>* positions, std::vector<float>* colors,
                                 size_t* occupied_voxels, std::string* err) const;

        double resolution() const { return resolution_; }
        const double* origin() const { return origin_; }
        int nx() const { return nx_; }
        int ny() const { return ny_; }
        int nz() const { return nz_; }

       private:
        struct QueueElement {
            fiesta::Vox3i point;
            double distance = 0.0;
        };

        bool initGrid(const RkvEsdfGridParams& params, const double min_b[3], const double max_b[3], std::string* err);
        void resetFiestaBuffers();
        void markOccupiedDirect(const std::vector<float>& xyz, size_t point_count);
        void integrateRaycast(const std::vector<float>& xyz, size_t point_count, const RkvEsdfGridParams& params);
        void raycastOnePoint(double px, double py, double pz, const RkvEsdfGridParams& params, int stamp);
        int setOccupancyWorld(double x, double y, double z, uint8_t occ);

        bool voxInRange(const fiesta::Vox3i& v) const;
        int voxToIndex(const fiesta::Vox3i& v) const;
        int closestLinkIndex(const fiesta::Vox3i& v) const;
        int closestLinkIndex(int idx) const;
        fiesta::Vox3i indexToVox(int idx) const;
        bool fiestaExist(int idx) const;
        double fiestaDist(const fiesta::Vox3i& a, const fiesta::Vox3i& b) const;
        fiesta::Vox3i closestAt(int idx) const;
        void setClosestAt(int idx, const fiesta::Vox3i& v);
        void deleteFromList(int link, int idx);
        void insertIntoList(int link, int idx);
        int setOccupancyVox(const fiesta::Vox3i& vox, uint8_t occ);
        void updateEsdfFiesta();

        int index(int x, int y, int z) const;
        void posToVox(double x, double y, double z, int* vx, int* vy, int* vz) const;
        void voxToPos(int vx, int vy, int vz, float* px, float* py, float* pz) const;
        bool distanceValid(double d) const;

        double resolution_          = 0.05;
        double origin_[3]           = {};
        int nx_                     = 0;
        int ny_                     = 0;
        int nz_                     = 0;
        int nxy_                    = 0;
        int grid_total_size_        = 0;
        int reserved_idx_undefined_ = 0;

        std::vector<uint8_t> occupancy_buffer_;
        std::vector<double> distance_buffer_;
        std::vector<int> closest_obstacle_;
        std::vector<int> head_;
        std::vector<int> prev_;
        std::vector<int> next_;

        std::queue<QueueElement> insert_queue_;
        std::queue<QueueElement> delete_queue_;
        std::queue<QueueElement> update_queue_;

        std::vector<int> free_stamp_;
        std::vector<int> occ_stamp_;
        std::vector<RkvVec3d> ray_voxels_scratch_;
    };

}  // namespace kinematic_viewer
