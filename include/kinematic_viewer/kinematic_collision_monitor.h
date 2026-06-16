#pragma once

#include "kinematic_viewer/kinematic_collision_distance.h"
#include "kinematic_viewer/kinematic_runtime_state.h"
#include "rkv/scene.h"

#include <memory>

namespace kinematic_viewer {

    struct CollisionMonitorResult {
        bool valid               = false;
        int evaluated_pair_count = 0;
        int warning_pair_count   = 0;
        int danger_pair_count    = 0;
        CollisionPairDistance closest_pair;
    };

    class CollisionPairFilterStrategy {
       public:
        virtual ~CollisionPairFilterStrategy() = default;
        virtual bool ShouldEvaluate(const CollisionMonitorState& state, const rkv::RobotScene& scene,
                                    const rkv::RobotScene::LinkCollisionProxy& a, const rkv::RobotScene::LinkCollisionProxy& b) const = 0;
    };

    class DefaultCollisionPairFilterStrategy : public CollisionPairFilterStrategy {
       public:
        bool ShouldEvaluate(const CollisionMonitorState& state, const rkv::RobotScene& scene, const rkv::RobotScene::LinkCollisionProxy& a,
                            const rkv::RobotScene::LinkCollisionProxy& b) const override;
    };

    class CollisionMonitor {
       public:
        CollisionMonitor();

        void SetPairFilterStrategy(std::unique_ptr<CollisionPairFilterStrategy> strategy);
        CollisionMonitorResult Evaluate(const CollisionMonitorState& state, const rkv::RobotScene& scene) const;
        void UpdateStateFromResult(const CollisionMonitorResult& result, CollisionMonitorState* state) const;

       private:
        std::unique_ptr<CollisionPairFilterStrategy> pair_filter_strategy_;
    };

}  // namespace kinematic_viewer
