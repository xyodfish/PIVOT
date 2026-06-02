// FIESTA ESDF update — logic ported from aphropm_cplanner ESDFMap.cpp
// (dense grid, non-PROBABILISTIC, non-HASH_TABLE).

#include "kinematic_viewer/rkv_esdf_grid.h"
#include "kinematic_viewer/rkv_fiesta_dirs.h"

#include <cmath>
#include <queue>
#include <vector>

namespace kinematic_viewer {
    namespace {

        using fiesta::kDirs;
        using fiesta::kNumDirs;
        using fiesta::Vox3i;

        constexpr int kUndefined = -10000;
        constexpr int kInfinity  = 10000;

    }  // namespace

    void RkvEsdfGrid::resetFiestaBuffers() {
        const size_t total = static_cast<size_t>(grid_total_size_);
        occupancy_buffer_.assign(total, 0);
        distance_buffer_.assign(total, static_cast<double>(kInfinity));
        closest_obstacle_.assign(total * 3, kUndefined);
        head_.assign(total + 1, kUndefined);
        prev_.assign(total, kUndefined);
        next_.assign(total, kUndefined);

        while (!insert_queue_.empty()) {
            insert_queue_.pop();
        }
        while (!delete_queue_.empty()) {
            delete_queue_.pop();
        }
        while (!update_queue_.empty()) {
            update_queue_.pop();
        }
    }

    bool RkvEsdfGrid::voxInRange(const Vox3i& v) const {
        return v.x >= 0 && v.x < nx_ && v.y >= 0 && v.y < ny_ && v.z >= 0 && v.z < nz_;
    }

    int RkvEsdfGrid::voxToIndex(const Vox3i& v) const {
        return v.z * nxy_ + v.y * nx_ + v.x;
    }

    int RkvEsdfGrid::closestLinkIndex(const Vox3i& v) const {
        if (v.x == kUndefined) {
            return reserved_idx_undefined_;
        }
        return voxToIndex(v);
    }

    int RkvEsdfGrid::closestLinkIndex(int idx) const {
        return closestLinkIndex(closestAt(idx));
    }

    Vox3i RkvEsdfGrid::indexToVox(int idx) const {
        const int z   = idx / nxy_;
        const int rem = idx - z * nxy_;
        const int y   = rem / nx_;
        const int x   = rem % nx_;
        return {x, y, z};
    }

    bool RkvEsdfGrid::fiestaExist(int idx) const {
        return occupancy_buffer_[static_cast<size_t>(idx)] == 1;
    }

    double RkvEsdfGrid::fiestaDist(const Vox3i& a, const Vox3i& b) const {
        const double dx = static_cast<double>(b.x - a.x);
        const double dy = static_cast<double>(b.y - a.y);
        const double dz = static_cast<double>(b.z - a.z);
        return std::sqrt(dx * dx + dy * dy + dz * dz) * resolution_;
    }

    Vox3i RkvEsdfGrid::closestAt(int idx) const {
        const size_t o = static_cast<size_t>(idx) * 3;
        return {closest_obstacle_[o + 0], closest_obstacle_[o + 1], closest_obstacle_[o + 2]};
    }

    void RkvEsdfGrid::setClosestAt(int idx, const Vox3i& v) {
        const size_t o           = static_cast<size_t>(idx) * 3;
        closest_obstacle_[o + 0] = v.x;
        closest_obstacle_[o + 1] = v.y;
        closest_obstacle_[o + 2] = v.z;
    }

    void RkvEsdfGrid::deleteFromList(int link, int idx) {
        if (prev_[static_cast<size_t>(idx)] != kUndefined) {
            next_[static_cast<size_t>(prev_[static_cast<size_t>(idx)])] = next_[static_cast<size_t>(idx)];
        } else {
            head_[static_cast<size_t>(link)] = next_[static_cast<size_t>(idx)];
        }
        if (next_[static_cast<size_t>(idx)] != kUndefined) {
            prev_[static_cast<size_t>(next_[static_cast<size_t>(idx)])] = prev_[static_cast<size_t>(idx)];
        }
        prev_[static_cast<size_t>(idx)] = kUndefined;
        next_[static_cast<size_t>(idx)] = kUndefined;
    }

    void RkvEsdfGrid::insertIntoList(int link, int idx) {
        if (head_[static_cast<size_t>(link)] == kUndefined) {
            head_[static_cast<size_t>(link)] = idx;
        } else {
            prev_[static_cast<size_t>(head_[static_cast<size_t>(link)])] = idx;
            next_[static_cast<size_t>(idx)]                              = head_[static_cast<size_t>(link)];
            head_[static_cast<size_t>(link)]                             = idx;
        }
    }

    int RkvEsdfGrid::setOccupancyVox(const Vox3i& vox, uint8_t occ) {
        if (!voxInRange(vox)) {
            return kUndefined;
        }
        const int idx = voxToIndex(vox);
        if (occupancy_buffer_[static_cast<size_t>(idx)] != occ) {
            if (occ == 1) {
                insert_queue_.push(QueueElement{vox, 0.0});
            } else {
                delete_queue_.push(QueueElement{vox, static_cast<double>(kInfinity)});
            }
        }
        occupancy_buffer_[static_cast<size_t>(idx)] = occ;
        if (distance_buffer_[static_cast<size_t>(idx)] < 0) {
            distance_buffer_[static_cast<size_t>(idx)] = static_cast<double>(kInfinity);
            insertIntoList(reserved_idx_undefined_, idx);
        }
        return idx;
    }

    void RkvEsdfGrid::updateEsdfFiesta() {
        while (!insert_queue_.empty()) {
            QueueElement xx = insert_queue_.front();
            insert_queue_.pop();
            const int idx = voxToIndex(xx.point);
            if (!fiestaExist(idx)) {
                continue;
            }
            deleteFromList(closestLinkIndex(idx), idx);
            setClosestAt(idx, xx.point);
            distance_buffer_[static_cast<size_t>(idx)] = 0.0;
            insertIntoList(idx, idx);
            update_queue_.push(QueueElement{xx.point, 0.0});
        }

        while (!delete_queue_.empty()) {
            QueueElement xx = delete_queue_.front();
            delete_queue_.pop();
            const int idx = voxToIndex(xx.point);
            if (fiestaExist(idx)) {
                continue;
            }

            int next_obs_idx = 0;
            for (int obs_idx = head_[static_cast<size_t>(idx)]; obs_idx != kUndefined; obs_idx = next_obs_idx) {
                setClosestAt(obs_idx, {kUndefined, kUndefined, kUndefined});
                const Vox3i obs_vox = indexToVox(obs_idx);

                double distance = static_cast<double>(kInfinity);
                for (int d = 0; d < kNumDirs; ++d) {
                    const Vox3i new_pos{obs_vox.x + kDirs[d].x, obs_vox.y + kDirs[d].y, obs_vox.z + kDirs[d].z};
                    if (!voxInRange(new_pos)) {
                        continue;
                    }
                    const int new_pos_idx = voxToIndex(new_pos);
                    const Vox3i co        = closestAt(new_pos_idx);
                    if (co.x == kUndefined) {
                        continue;
                    }
                    if (!fiestaExist(voxToIndex(co))) {
                        continue;
                    }
                    const double tmp = fiestaDist(obs_vox, co);
                    if (tmp < distance) {
                        distance = tmp;
                        setClosestAt(obs_idx, co);
                    }
                    break;
                }

                prev_[static_cast<size_t>(obs_idx)] = kUndefined;
                next_obs_idx                        = next_[static_cast<size_t>(obs_idx)];
                next_[static_cast<size_t>(obs_idx)] = kUndefined;

                distance_buffer_[static_cast<size_t>(obs_idx)] = distance;
                if (distance < static_cast<double>(kInfinity)) {
                    update_queue_.push(QueueElement{obs_vox, distance});
                }
                const int new_obs_idx = closestLinkIndex(obs_idx);
                insertIntoList(new_obs_idx, obs_idx);
            }
            head_[static_cast<size_t>(idx)] = kUndefined;
        }

        while (!update_queue_.empty()) {
            QueueElement xx = update_queue_.front();
            update_queue_.pop();
            const int idx = voxToIndex(xx.point);
            if (xx.distance != distance_buffer_[static_cast<size_t>(idx)]) {
                continue;
            }

            bool change = false;
            for (int i = 0; i < kNumDirs; ++i) {
                const Vox3i new_pos{xx.point.x + kDirs[i].x, xx.point.y + kDirs[i].y, xx.point.z + kDirs[i].z};
                if (!voxInRange(new_pos)) {
                    continue;
                }
                const int new_pos_idx = voxToIndex(new_pos);
                if (closestAt(new_pos_idx).x == kUndefined) {
                    continue;
                }
                const double tmp = fiestaDist(xx.point, closestAt(new_pos_idx));
                if (distance_buffer_[static_cast<size_t>(idx)] > tmp) {
                    distance_buffer_[static_cast<size_t>(idx)] = tmp;
                    change                                     = true;
                    deleteFromList(closestLinkIndex(idx), idx);
                    const int new_obs_idx = closestLinkIndex(new_pos_idx);
                    insertIntoList(new_obs_idx, idx);
                    setClosestAt(idx, closestAt(new_pos_idx));
                }
            }

            if (change) {
                update_queue_.push(QueueElement{xx.point, distance_buffer_[static_cast<size_t>(idx)]});
                continue;
            }

            const int new_obs_idx = closestLinkIndex(idx);
            for (int i = 0; i < kNumDirs; ++i) {
                const Vox3i new_pos{xx.point.x + kDirs[i].x, xx.point.y + kDirs[i].y, xx.point.z + kDirs[i].z};
                if (!voxInRange(new_pos)) {
                    continue;
                }
                const int new_pos_id = voxToIndex(new_pos);
                const double tmp     = fiestaDist(new_pos, closestAt(idx));
                if (distance_buffer_[static_cast<size_t>(new_pos_id)] > tmp) {
                    distance_buffer_[static_cast<size_t>(new_pos_id)] = tmp;
                    deleteFromList(closestLinkIndex(new_pos_id), new_pos_id);
                    insertIntoList(new_obs_idx, new_pos_id);
                    setClosestAt(new_pos_id, closestAt(idx));
                    update_queue_.push(QueueElement{new_pos, tmp});
                }
            }
        }
    }

}  // namespace kinematic_viewer
