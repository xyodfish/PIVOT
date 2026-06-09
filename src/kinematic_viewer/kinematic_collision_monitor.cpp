#include "kinematic_viewer/kinematic_collision_monitor.h"

#include "kinematic_viewer/kinematic_collision_distance.h"

#include <algorithm>
#include <limits>
#include <unordered_map>
#include <vector>

namespace kinematic_viewer {
    namespace kinematic_collision_monitor_internal {

        constexpr float kMeshRefineAabbDistanceM = 0.12f;
        constexpr int kSameChainActiveJointLimit   = 2;
        constexpr size_t kMaxMeshRefinePairsPerEval = 3;

        bool IsFixedJointToParent(const rkv::RobotScene& scene, const std::string& link_name) {
            std::string joint_name;
            if (!scene.getParentJointNameForLink(link_name, &joint_name)) {
                return false;
            }
            rkv::RobotScene::JointDetailInfo detail;
            if (!scene.getJointDetail(joint_name, &detail)) {
                return false;
            }
            return detail.type == "fixed";
        }

        bool IsAncestorOf(const rkv::RobotScene& scene, const std::string& ancestor, const std::string& descendant) {
            std::string current = descendant;
            while (true) {
                std::string parent;
                if (!scene.getLinkParentName(current, &parent) || parent.empty()) {
                    return false;
                }
                if (parent == ancestor) {
                    return true;
                }
                current = parent;
            }
        }

        int CountActiveJointsOnPath(const rkv::RobotScene& scene, const std::string& descendant,
                                    const std::string& ancestor) {
            int active_joint_count = 0;
            std::string current    = descendant;
            while (current != ancestor) {
                if (!IsFixedJointToParent(scene, current)) {
                    ++active_joint_count;
                }
                std::string parent;
                if (!scene.getLinkParentName(current, &parent)) {
                    return std::numeric_limits<int>::max();
                }
                current = parent;
            }
            return active_joint_count;
        }

        bool AreKinematicallyConnectedLinks(const rkv::RobotScene& scene, const std::string& a, const std::string& b) {
            if (IsAncestorOf(scene, a, b)) {
                return CountActiveJointsOnPath(scene, b, a) <= kSameChainActiveJointLimit;
            }
            if (IsAncestorOf(scene, b, a)) {
                return CountActiveJointsOnPath(scene, a, b) <= kSameChainActiveJointLimit;
            }
            return false;
        }

        void MergeProxyAabb(const rkv::RobotScene::LinkCollisionProxy& src,
                            rkv::RobotScene::LinkCollisionProxy* dst) {
            if (dst == nullptr || !src.has_world_aabb) {
                return;
            }
            if (!dst->has_world_aabb) {
                dst->world_aabb_min = src.world_aabb_min;
                dst->world_aabb_max = src.world_aabb_max;
                dst->has_world_aabb = true;
                return;
            }
            dst->world_aabb_min = glm::min(dst->world_aabb_min, src.world_aabb_min);
            dst->world_aabb_max = glm::max(dst->world_aabb_max, src.world_aabb_max);
        }

        std::vector<rkv::RobotScene::LinkCollisionProxy> BuildUniqueLinkProxies(
            const std::vector<rkv::RobotScene::LinkCollisionProxy>& proxies) {
            std::vector<rkv::RobotScene::LinkCollisionProxy> unique_proxies;
            std::unordered_map<std::string, size_t> link_index;
            unique_proxies.reserve(proxies.size());

            for (const auto& proxy : proxies) {
                const auto it = link_index.find(proxy.link_name);
                if (it == link_index.end()) {
                    link_index.emplace(proxy.link_name, unique_proxies.size());
                    unique_proxies.push_back(proxy);
                    continue;
                }
                MergeProxyAabb(proxy, &unique_proxies[it->second]);
            }
            return unique_proxies;
        }

    }  // namespace kinematic_collision_monitor_internal

    bool DefaultCollisionPairFilterStrategy::ShouldEvaluate(const CollisionMonitorState& state, const rkv::RobotScene& scene,
                                                            const rkv::RobotScene::LinkCollisionProxy& a,
                                                            const rkv::RobotScene::LinkCollisionProxy& b) const {
        if (state.ignore_same_link && a.link_name == b.link_name) {
            return false;
        }

        if (state.ignore_parent_child) {
            if (kinematic_collision_monitor_internal::AreKinematicallyConnectedLinks(scene, a.link_name, b.link_name)) {
                return false;
            }
        }

        return true;
    }

    CollisionMonitor::CollisionMonitor() : pair_filter_strategy_(std::make_unique<DefaultCollisionPairFilterStrategy>()) {}

    void CollisionMonitor::SetPairFilterStrategy(std::unique_ptr<CollisionPairFilterStrategy> strategy) {
        if (strategy) {
            pair_filter_strategy_ = std::move(strategy);
        }
    }

    CollisionMonitorResult CollisionMonitor::Evaluate(const CollisionMonitorState& state, const rkv::RobotScene& scene) const {
        CollisionMonitorResult result;
        if (!state.enable || !pair_filter_strategy_) {
            return result;
        }

        const auto unique_proxies = kinematic_collision_monitor_internal::BuildUniqueLinkProxies(scene.getLinkCollisionProxies());
        if (unique_proxies.size() < 2) {
            return result;
        }

        struct PairCandidate {
            CollisionPairDistance aabb_distance;
        };

        std::vector<PairCandidate> candidates;
        candidates.reserve(unique_proxies.size() * unique_proxies.size() / 2);

        for (size_t i = 0; i < unique_proxies.size(); ++i) {
            for (size_t j = i + 1; j < unique_proxies.size(); ++j) {
                const auto& a = unique_proxies[i];
                const auto& b = unique_proxies[j];
                if (!pair_filter_strategy_->ShouldEvaluate(state, scene, a, b)) {
                    continue;
                }
                ++result.evaluated_pair_count;
                candidates.push_back({BuildCollisionPairDistanceAabb(a, b)});
            }
        }

        if (candidates.empty()) {
            return result;
        }

        std::sort(candidates.begin(), candidates.end(), [](const PairCandidate& lhs, const PairCandidate& rhs) {
            return lhs.aabb_distance.surface_distance_m < rhs.aabb_distance.surface_distance_m;
        });

        const float refine_threshold_m =
            std::max(kinematic_collision_monitor_internal::kMeshRefineAabbDistanceM, state.warning_distance_m * 3.0f);
        const float mesh_refine_cutoff_m =
            std::min(refine_threshold_m, candidates.front().aabb_distance.surface_distance_m + 0.02f);

        size_t mesh_refines_used = 0;
        bool has_closest         = false;

        for (size_t index = 0; index < candidates.size(); ++index) {
            const PairCandidate& candidate = candidates[index];
            CollisionPairDistance distance = candidate.aabb_distance;

            const bool should_refine_mesh =
                mesh_refines_used < kinematic_collision_monitor_internal::kMaxMeshRefinePairsPerEval &&
                (index < kinematic_collision_monitor_internal::kMaxMeshRefinePairsPerEval ||
                 distance.surface_distance_m <= mesh_refine_cutoff_m || distance.surface_distance_m <= 1e-4f);

            if (should_refine_mesh) {
                distance = RefineCollisionPairDistanceWithMesh(scene, distance);
                ++mesh_refines_used;
            }

            if (distance.surface_distance_m <= state.warning_distance_m) {
                ++result.warning_pair_count;
            }
            if (distance.surface_distance_m <= state.danger_distance_m) {
                ++result.danger_pair_count;
            }

            if (!has_closest || distance.surface_distance_m < result.closest_pair.surface_distance_m) {
                result.closest_pair = std::move(distance);
                has_closest         = true;
            }
        }

        result.valid = has_closest;
        return result;
    }

    void CollisionMonitor::UpdateStateFromResult(const CollisionMonitorResult& result, CollisionMonitorState* state) const {
        if (state == nullptr) {
            return;
        }

        state->evaluated_pair_count = result.evaluated_pair_count;
        state->has_valid_distance   = result.valid;
        if (!result.valid) {
            state->nearest_link_a.clear();
            state->nearest_link_b.clear();
            state->nearest_surface_distance_m = 0.0f;
            state->nearest_center_distance_m  = 0.0f;
            state->nearest_point_a            = glm::vec3(0.0f);
            state->nearest_point_b            = glm::vec3(0.0f);
            return;
        }

        state->nearest_link_a             = result.closest_pair.link_a;
        state->nearest_link_b             = result.closest_pair.link_b;
        state->nearest_surface_distance_m = result.closest_pair.surface_distance_m;
        state->nearest_center_distance_m  = result.closest_pair.center_distance_m;
        state->nearest_point_a            = result.closest_pair.point_a;
        state->nearest_point_b            = result.closest_pair.point_b;
    }

}  // namespace kinematic_viewer
