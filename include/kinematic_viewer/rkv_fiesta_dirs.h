#pragma once

// 24-neighbor offsets ( parameters.h num_dirs_ = 24)
namespace kinematic_viewer::fiesta {

    struct Vox3i {
        int x = 0;
        int y = 0;
        int z = 0;
    };

    inline constexpr int kNumDirs = 24;

    inline const Vox3i kDirs[kNumDirs] = {
        {-1, 0, 0},  {1, 0, 0},  {0, -1, 0},  {0, 1, 0}, {0, 0, -1}, {0, 0, 1},  {-1, -1, 0}, {1, 1, 0},
        {0, -1, -1}, {0, 1, 1},  {-1, 0, -1}, {1, 0, 1}, {-1, 1, 0}, {1, -1, 0}, {0, -1, 1},  {0, 1, -1},
        {1, 0, -1},  {-1, 0, 1}, {-2, 0, 0},  {2, 0, 0}, {0, -2, 0}, {0, 2, 0},  {0, 0, -2},  {0, 0, 2},
    };

}  // namespace kinematic_viewer::fiesta
