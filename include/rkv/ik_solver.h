#pragma once

#include "rkv/config_types.h"
#include "rkv/scene.h"

#include <flex_ik.h>
#include <glm/glm.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include <wbc/robot/traits/chain_ik.h>
#include <trac_ik/trac_ik.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace rkv {

    struct IkChainStatus {
        ViewerIkChainConfig config;
        bool ready = false;
        std::string error;
    };

    struct IkSolveStats {
        bool success                = false;
        int solved_chain_count      = 0;
        int ready_chain_count       = 0;
        bool active_chain_solved    = false;
        float max_position_error_mm = 0.0f;
    };

    class IkSolver {
       public:
        void setFullBodyBackend(const std::string& backendName);
        const std::string& fullBodyBackend() const;

        bool initialize(const std::string& urdfPath, const std::vector<ViewerIkChainConfig>& chains);

        int chainCount() const;
        const IkChainStatus& chainStatus(int chainIndex) const;

        bool fetchTipWorldPose(const RobotScene& scene, int chainIndex, glm::vec3* worldPos, glm::vec3* worldRpy) const;

        bool solveSingleChain(RobotScene* scene, int chainIndex, const glm::mat4& targetWorld, std::string* statusText);

        /**
         * @brief Query IK solution for a single chain without modifying the scene.
         * @param chainIndex Index of the chain to solve
         * @param targetWorld Target transform in world coordinates
         * @param baseWorld Base link transform in world coordinates (used to convert target to base frame)
         * @param seedJointPositions Seed joint positions (must match chain joint count). If empty, uses neutral pose.
         * @param outSolution Output joint positions on success
         * @param outStatusText Optional status message
         * @return true if IK succeeded
         */
        bool querySingleChain(int chainIndex, const glm::mat4& targetWorld, const glm::mat4& baseWorld,
                              const std::vector<float>& seedJointPositions, std::vector<float>* outSolution, std::string* statusText) const;

        bool solveFullBody(RobotScene* scene, const std::vector<glm::mat4>& targetWorldByChain, int iterations, int activeChainIndex,
                           bool fastMode, bool positionOnlyMode, IkSolveStats* stats, std::string* statusText);

       private:
        struct IkChainRuntime {
            IkChainStatus status;
            std::unique_ptr<TRAC_IK::TRAC_IK> solver;
            KDL::Chain chain;
            KDL::JntArray lower;
            KDL::JntArray upper;
            std::vector<std::string> jointNames;
        };

        KDL::JntArray buildSeedFromScene(const RobotScene& scene, const IkChainRuntime& chainRuntime) const;

        bool solveChainInternal(RobotScene* scene, int chainIndex, const IkChainRuntime& chainRuntime, const glm::mat4& targetWorld,
                                int maxRetryCount, float* outPositionErrorMm);

        bool findLinkWorldTransform(const RobotScene& scene, const std::string& linkName, glm::mat4* worldTransform,
                                    glm::vec3* worldRpy) const;

        static KDL::Frame glmToKdlFrame(const glm::mat4& transform);
        static flex_ik::SE3 glmToFlexSe3(const glm::mat4& transform);

        bool initializeFullBodySolvers(const std::string& urdfPath, const std::vector<ViewerIkChainConfig>& chains);
        bool loadUrdfLinkNames(const std::string& urdfPath, std::unordered_set<std::string>* linkNames, std::string* errorText) const;
        bool pickFirstExisting(const std::vector<std::string>& candidates, const std::unordered_set<std::string>& linkNames,
                               std::string* resolvedLink) const;
        ViewerIkChainConfig resolveChainConfigFromUrdf(const ViewerIkChainConfig& config, const std::unordered_set<std::string>& linkNames,
                                                       bool* resolved, std::string* reason) const;
        flex_ik::Vector buildFullBodyQFromScene(const RobotScene& scene) const;
        void applyFullBodyQToScene(RobotScene* scene, const flex_ik::Vector& q) const;
        flex_ik::Vector buildWbcFullBodyQFromScene(const RobotScene& scene) const;
        void applyWbcFullBodyQToScene(RobotScene* scene, const flex_ik::Vector& q) const;
        flex_ik::Vector limitFullBodyStep(const flex_ik::Vector& qCurrent, const flex_ik::Vector& qSolved, bool fastMode,
                                          bool positionOnlyMode) const;
        flex_ik::Vector limitWbcFullBodyStep(const flex_ik::Vector& qCurrent, const flex_ik::Vector& qSolved, bool fastMode,
                                             bool positionOnlyMode, bool lockFloatingBase) const;

        std::vector<IkChainRuntime> chains_;
        std::unique_ptr<flex_ik::FlexIk> fullBodySolverPose_;
        std::unique_ptr<flex_ik::FlexIk> fullBodySolverPosOnly_;
        std::unique_ptr<Wbc::robot::ChainIkTrait> fullBodyWbcSolverPose_;
        std::unique_ptr<Wbc::robot::ChainIkTrait> fullBodyWbcSolverPosOnly_;
        bool fullBodyFlexReady_ = false;
        bool fullBodyWbcReady_  = false;
        bool fullBodyReady_     = false;
        std::string fullBodyError_;
        std::string fullBodyBackend_ = "flex_ik";
        std::unordered_map<std::string, int> fullBodyJointQIndex_;
        std::vector<int> fullBodyLockedVIndex_;
        std::unordered_map<std::string, int> fullBodyWbcJointQIndex_;
        std::vector<int> fullBodyWbcLockedVIndex_;
        std::vector<int> fullBodyWbcPlanarBaseVIndex_;
        int fullBodyWbcNv_               = 0;
        bool fullBodyWbcHasFloatingBase_ = false;
    };

}  // namespace rkv
