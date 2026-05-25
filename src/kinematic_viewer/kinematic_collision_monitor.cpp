#include "kinematic_viewer/kinematic_collision_monitor.h"

#include <algorithm>

namespace kinematic_viewer {
    namespace kinematic_collision_monitor_internal {

        CollisionPairDistance BuildDistance(const teleop_viewer::RobotScene::LinkCollisionProxy& a,
                                            const teleop_viewer::RobotScene::LinkCollisionProxy& b) {
            CollisionPairDistance result;
            result.link_a = a.link_name;
            result.link_b = b.link_name;

            const glm::vec3 delta       = b.world_center - a.world_center;
            const float center_distance = glm::length(delta);
            result.center_distance_m    = center_distance;

            glm::vec3 direction(1.0f, 0.0f, 0.0f);
            if (center_distance > 1e-6f) {
                direction = delta / center_distance;
            }

            result.point_a            = a.world_center + direction * a.radius_m;
            result.point_b            = b.world_center - direction * b.radius_m;
            result.surface_distance_m = center_distance - (a.radius_m + b.radius_m);
            return result;
        }

    }  // namespace kinematic_collision_monitor_internal

    bool DefaultCollisionPairFilterStrategy::ShouldEvaluate(const CollisionMonitorState& state,
                                                            const teleop_viewer::RobotScene& scene,
                                                            const teleop_viewer::RobotScene::LinkCollisionProxy& a,
                                                            const teleop_viewer::RobotScene::LinkCollisionProxy& b) const {
        if (state.ignore_same_link && a.link_name == b.link_name) {
            return false;
        }

        if (state.ignore_parent_child) {
            std::string a_parent;
            if (scene.getLinkParentName(a.link_name, &a_parent) && a_parent == b.link_name) {
                return false;
            }
            std::string b_parent;
            if (scene.getLinkParentName(b.link_name, &b_parent) && b_parent == a.link_name) {
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

    CollisionMonitorResult CollisionMonitor::Evaluate(const CollisionMonitorState& state,
                                                      const teleop_viewer::RobotScene& scene) const {
        CollisionMonitorResult result;
        if (!state.enable || !pair_filter_strategy_) {
            return result;
        }

        const auto proxies = scene.getLinkCollisionProxies();
        if (proxies.size() < 2) {
            return result;
        }

        bool has_closest = false;
        for (size_t i = 0; i < proxies.size(); ++i) {
            for (size_t j = i + 1; j < proxies.size(); ++j) {
                const auto& a = proxies[i];
                const auto& b = proxies[j];
                if (!pair_filter_strategy_->ShouldEvaluate(state, scene, a, b)) {
                    continue;
                }
                ++result.evaluated_pair_count;
                CollisionPairDistance distance = kinematic_collision_monitor_internal::BuildDistance(a, b);
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
