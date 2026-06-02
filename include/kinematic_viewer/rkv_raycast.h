#pragma once

// Voxel ray traversal (Amanatides & Woo, 1987), logic aligned with
// aphropm_cplanner/src/aphropm_plan_env/esdf_map/raycast.cpp

#include <vector>

namespace kinematic_viewer {

    struct RkvVec3d {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    void RkvRaycast(const RkvVec3d& start, const RkvVec3d& end, const RkvVec3d& grid_min, const RkvVec3d& grid_max,
                    std::vector<RkvVec3d>* output);

}  // namespace kinematic_viewer
