#pragma once

#include "kinematic_viewer/kinematic_runtime_state.h"
#include "teleop_viewer/scene.h"

#include <memory>

namespace kinematic_viewer {

    struct CollisionPairDistance {
        std::string link_a;
        std::string link_b;
        glm::vec3 point_a        = glm::vec3(0.0f);
        glm::vec3 point_b        = glm::vec3(0.0f);
        float center_distance_m  = 0.0f;
        float surface_distance_m = 0.0f;
    };

    struct CollisionMonitorResult {
        bool valid               = false;
        int evaluated_pair_count = 0;
        int warning_pair_count   = 0;
        int danger_pair_count    = 0;
        CollisionPairDistance closest_pair;
    };

    class CollisionPairFilterStrategy {
       public:
        virtual ~CollisionPairFilterStrategy()                                                              = default;
        virtual bool ShouldEvaluate(const CollisionMonitorState& state, const teleop_viewer::RobotScene& scene,
                                    const teleop_viewer::RobotScene::LinkCollisionProxy& a,
                                    const teleop_viewer::RobotScene::LinkCollisionProxy& b) const = 0;
    };

    class DefaultCollisionPairFilterStrategy : public CollisionPairFilterStrategy {
       public:
        bool ShouldEvaluate(const CollisionMonitorState& state, const teleop_viewer::RobotScene& scene,
                            const teleop_viewer::RobotScene::LinkCollisionProxy& a,
                            const teleop_viewer::RobotScene::LinkCollisionProxy& b) const override;
    };

    class CollisionMonitor {
       public:
        CollisionMonitor();

        void SetPairFilterStrategy(std::unique_ptr<CollisionPairFilterStrategy> strategy);
        CollisionMonitorResult Evaluate(const CollisionMonitorState& state, const teleop_viewer::RobotScene& scene) const;
        void UpdateStateFromResult(const CollisionMonitorResult& result, CollisionMonitorState* state) const;

       private:
        std::unique_ptr<CollisionPairFilterStrategy> pair_filter_strategy_;
    };

}  // namespace kinematic_viewer
