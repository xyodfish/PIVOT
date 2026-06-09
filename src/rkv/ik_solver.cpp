#include "rkv/ik_solver.h"

#include <urdf_parser/urdf_parser.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/multibody/joint/joint-free-flyer.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>
#include <sstream>
#include <stdexcept>

namespace rkv {
    namespace {

        pinocchio::Model BuildFloatingBaseModelForWbc(const std::string& urdfPath, bool* hasFloatingBase) {
            if (hasFloatingBase != nullptr) {
                *hasFloatingBase = false;
            }
            pinocchio::Model model;
            pinocchio::JointModelFreeFlyer rootJoint;
            pinocchio::urdf::buildModel(urdfPath, rootJoint, model);
            if (hasFloatingBase != nullptr) {
                *hasFloatingBase = (model.nq >= 7 && model.nv >= 6);
            }
            return model;
        }

        void virtualBase2DToFloatingBaseQ(float x_m, float y_m, float yaw_rad, Eigen::VectorXd* q) {
            if (q == nullptr || q->size() < 7) {
                return;
            }
            const Eigen::AngleAxisd yawAxis(static_cast<double>(yaw_rad), Eigen::Vector3d::UnitZ());
            const Eigen::Quaterniond quat(yawAxis);
            (*q)[0] = static_cast<double>(x_m);
            (*q)[1] = static_cast<double>(y_m);
            (*q)[2] = 0.0;
            (*q)[3] = quat.x();
            (*q)[4] = quat.y();
            (*q)[5] = quat.z();
            (*q)[6] = quat.w();
        }

        bool floatingBaseQToVirtualBase2D(const Eigen::VectorXd& q, float* x_m, float* y_m, float* yaw_rad) {
            if (q.size() < 7 || x_m == nullptr || y_m == nullptr || yaw_rad == nullptr) {
                return false;
            }
            *x_m            = static_cast<float>(q[0]);
            *y_m            = static_cast<float>(q[1]);
            const double qx = q[3];
            const double qy = q[4];
            const double qz = q[5];
            const double qw = q[6];
            *yaw_rad        = static_cast<float>(std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)));
            return true;
        }

        std::vector<Wbc::Index> buildWbcFixedVelocityIndices(const std::vector<int>& lockedVIndex, const std::vector<int>& planarBaseVIndex,
                                                             bool lockPlanarBase) {
            std::vector<Wbc::Index> fixed;
            fixed.reserve(lockedVIndex.size());
            for (const int idx : lockedVIndex) {
                if (!lockPlanarBase) {
                    const bool isPlanar = std::find(planarBaseVIndex.begin(), planarBaseVIndex.end(), idx) != planarBaseVIndex.end();
                    if (isPlanar) {
                        continue;
                    }
                }
                fixed.push_back(static_cast<Wbc::Index>(idx));
            }
            return fixed;
        }

    }  // namespace

    void IkSolver::setFullBodyBackend(const std::string& backendName) {
        if (backendName == "wbc_chain_ik") {
            fullBodyBackend_ = "wbc_chain_ik";
            return;
        }
        fullBodyBackend_ = "flex_ik";
    }

    const std::string& IkSolver::fullBodyBackend() const {
        return fullBodyBackend_;
    }

    bool IkSolver::loadUrdfLinkNames(const std::string& urdfPath, std::unordered_set<std::string>* linkNames,
                                     std::string* errorText) const {
        if (linkNames == nullptr) {
            if (errorText) {
                *errorText = "输出参数为空";
            }
            return false;
        }
        linkNames->clear();

        std::ifstream file(urdfPath);
        if (!file) {
            if (errorText) {
                *errorText = "无法打开URDF: " + urdfPath;
            }
            return false;
        }
        const std::string xml((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        auto model = urdf::parseURDF(xml);
        if (!model) {
            if (errorText) {
                *errorText = "URDF解析失败: " + urdfPath;
            }
            return false;
        }
        for (const auto& kv : model->links_) {
            linkNames->insert(kv.first);
        }
        if (linkNames->empty()) {
            if (errorText) {
                *errorText = "URDF中未找到link定义";
            }
            return false;
        }
        return true;
    }

    bool IkSolver::pickFirstExisting(const std::vector<std::string>& candidates, const std::unordered_set<std::string>& linkNames,
                                     std::string* resolvedLink) const {
        for (const auto& name : candidates) {
            if (name.empty()) {
                continue;
            }
            if (linkNames.find(name) == linkNames.end()) {
                continue;
            }
            if (resolvedLink != nullptr) {
                *resolvedLink = name;
            }
            return true;
        }
        return false;
    }

    ViewerIkChainConfig IkSolver::resolveChainConfigFromUrdf(const ViewerIkChainConfig& config,
                                                             const std::unordered_set<std::string>& linkNames, bool* resolved,
                                                             std::string* reason) const {
        ViewerIkChainConfig out = config;

        bool baseOk = !out.base_link.empty() && linkNames.find(out.base_link) != linkNames.end();
        if (!baseOk) {
            baseOk = pickFirstExisting(config.base_link_candidates, linkNames, &out.base_link);
        }

        bool tipOk = !out.tip_link.empty() && linkNames.find(out.tip_link) != linkNames.end();
        if (!tipOk) {
            tipOk = pickFirstExisting(config.tip_link_candidates, linkNames, &out.tip_link);
        }

        if (resolved != nullptr) {
            *resolved = baseOk && tipOk;
        }
        if (reason != nullptr && (!baseOk || !tipOk)) {
            std::ostringstream oss;
            if (!baseOk) {
                oss << "base_link不可解析";
            }
            if (!tipOk) {
                if (!oss.str().empty()) {
                    oss << ", ";
                }
                oss << "tip_link不可解析";
            }
            *reason = oss.str();
        }
        return out;
    }

    bool IkSolver::initialize(const std::string& urdfPath, const std::vector<ViewerIkChainConfig>& chains) {
        chains_.clear();
        chains_.reserve(chains.size());
        std::vector<ViewerIkChainConfig> fullBodyChains;
        fullBodyChains.reserve(chains.size());

        std::unordered_set<std::string> urdfLinkNames;
        std::string urdfLinkLoadError;
        const bool hasUrdfLinkNames = loadUrdfLinkNames(urdfPath, &urdfLinkNames, &urdfLinkLoadError);

        for (const auto& chain : chains) {
            ViewerIkChainConfig resolvedChain = chain;
            if (hasUrdfLinkNames) {
                bool chainResolved = false;
                std::string unresolvedReason;
                resolvedChain = resolveChainConfigFromUrdf(chain, urdfLinkNames, &chainResolved, &unresolvedReason);
                if (!chainResolved) {
                    IkChainRuntime runtime;
                    runtime.status.config = resolvedChain;
                    runtime.status.ready  = false;
                    runtime.status.error  = "IK链配置无效（未在URDF中解析到base/tip）: " + unresolvedReason;
                    chains_.push_back(std::move(runtime));
                    continue;
                }
            }

            if (resolvedChain.base_link.empty() || resolvedChain.tip_link.empty()) {
                IkChainRuntime runtime;
                runtime.status.config = resolvedChain;
                runtime.status.ready  = false;
                runtime.status.error  = "IK链配置缺失base_link或tip_link";
                chains_.push_back(std::move(runtime));
                continue;
            }

            IkChainRuntime runtime;
            runtime.status.config = resolvedChain;
            runtime.solver = std::make_unique<TRAC_IK::TRAC_IK>(resolvedChain.base_link, resolvedChain.tip_link, urdfPath, 200, 0.005, 1e-5,
                                                                1, false, false, TRAC_IK::Speed);
            if (!runtime.solver->getKDLChain(runtime.chain) || !runtime.solver->getKDLLimits(runtime.lower, runtime.upper)) {
                runtime.status.ready = false;
                runtime.status.error = "TRAC-IK 初始化失败: " + resolvedChain.base_link + " -> " + resolvedChain.tip_link;
                chains_.push_back(std::move(runtime));
                continue;
            }

            runtime.jointNames.clear();
            runtime.jointNames.reserve(static_cast<size_t>(runtime.chain.getNrOfJoints()));
            for (unsigned int i = 0; i < runtime.chain.getNrOfSegments(); ++i) {
                const auto& segment = runtime.chain.getSegment(i);
                if (segment.getJoint().getType() != KDL::Joint::None) {
                    runtime.jointNames.push_back(segment.getJoint().getName());
                }
            }

            if (runtime.jointNames.size() != static_cast<size_t>(runtime.chain.getNrOfJoints())) {
                runtime.status.ready = false;
                runtime.status.error = "关节名数量与链关节数不一致";
                chains_.push_back(std::move(runtime));
                continue;
            }

            runtime.status.ready = true;
            runtime.status.error.clear();
            fullBodyChains.push_back(runtime.status.config);
            chains_.push_back(std::move(runtime));
        }

        if (fullBodyChains.empty() && !hasUrdfLinkNames && !urdfLinkLoadError.empty()) {
            for (auto& chainRuntime : chains_) {
                if (chainRuntime.status.error.empty()) {
                    chainRuntime.status.error = "URDF link解析失败: " + urdfLinkLoadError;
                }
            }
        }

        initializeFullBodySolvers(urdfPath, fullBodyChains);
        return !chains_.empty();
    }

    int IkSolver::chainCount() const {
        return static_cast<int>(chains_.size());
    }

    const IkChainStatus& IkSolver::chainStatus(int chainIndex) const {
        static IkChainStatus invalid;
        if (chainIndex < 0 || chainIndex >= chainCount()) {
            return invalid;
        }
        return chains_[chainIndex].status;
    }

    bool IkSolver::findLinkWorldTransform(const RobotScene& scene, const std::string& linkName, glm::mat4* worldTransform,
                                          glm::vec3* worldRpy) const {
        if (worldTransform == nullptr) {
            return false;
        }
        if (!scene.getLinkWorldTransform(linkName, worldTransform)) {
            return false;
        }
        if (worldRpy != nullptr) {
            const glm::quat q = glm::quat_cast(*worldTransform);
            *worldRpy         = glm::eulerAngles(q);
        }
        return true;
    }

    bool IkSolver::fetchTipWorldPose(const RobotScene& scene, int chainIndex, glm::vec3* worldPos, glm::vec3* worldRpy) const {
        if (chainIndex < 0 || chainIndex >= chainCount()) {
            return false;
        }
        if (!worldPos || !worldRpy) {
            return false;
        }

        glm::mat4 worldTransform(1.0f);
        if (!findLinkWorldTransform(scene, chains_[chainIndex].status.config.tip_link, &worldTransform, worldRpy)) {
            return false;
        }
        *worldPos = glm::vec3(worldTransform[3]);
        return true;
    }

    KDL::Frame IkSolver::glmToKdlFrame(const glm::mat4& transform) {
        const glm::vec3 p = glm::vec3(transform[3]);
        const glm::quat q = glm::quat_cast(transform);
        return KDL::Frame(KDL::Rotation::Quaternion(q.x, q.y, q.z, q.w), KDL::Vector(p.x, p.y, p.z));
    }

    flex_ik::SE3 IkSolver::glmToFlexSe3(const glm::mat4& transform) {
        Eigen::Matrix3d rotation;
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                rotation(row, col) = static_cast<double>(transform[col][row]);
            }
        }
        const Eigen::Vector3d translation(static_cast<double>(transform[3][0]), static_cast<double>(transform[3][1]),
                                          static_cast<double>(transform[3][2]));
        return flex_ik::SE3(rotation, translation);
    }

    KDL::JntArray IkSolver::buildSeedFromScene(const RobotScene& scene, const IkChainRuntime& chainRuntime) const {
        const size_t jointCount = chainRuntime.jointNames.size();
        KDL::JntArray seed(jointCount);
        for (size_t i = 0; i < jointCount; ++i) {
            RobotScene::JointInfo info;
            if (scene.getJointInfo(chainRuntime.jointNames[i], &info)) {
                seed(i) = std::min(chainRuntime.upper(i), std::max(chainRuntime.lower(i), static_cast<double>(info.position)));
            } else {
                seed(i) = 0.5 * (chainRuntime.lower(i) + chainRuntime.upper(i));
            }
        }
        return seed;
    }

    bool IkSolver::solveChainInternal(RobotScene* scene, int chainIndex, const IkChainRuntime& chainRuntime, const glm::mat4& targetWorld,
                                      int maxRetryCount, float* outPositionErrorMm) {
        if (scene == nullptr) {
            return false;
        }

        glm::mat4 baseWorld(1.0f);
        if (!findLinkWorldTransform(*scene, chainRuntime.status.config.base_link, &baseWorld, nullptr)) {
            return false;
        }

        const glm::mat4 targetInBase = glm::inverse(baseWorld) * targetWorld;
        const KDL::Frame target      = glmToKdlFrame(targetInBase);
        KDL::JntArray seed           = buildSeedFromScene(*scene, chainRuntime);
        KDL::JntArray output(chainRuntime.jointNames.size());

        int rc = chainRuntime.solver->CartToJnt(seed, target, output);
        if (rc < 0 && maxRetryCount > 0) {
            for (int i = 0; i < maxRetryCount && rc < 0; ++i) {
                KDL::Frame perturbed = target;
                const double dp      = 0.004 * static_cast<double>(i + 1);
                perturbed.p.x(target.p.x() + dp);
                perturbed.p.y(target.p.y() - dp);
                rc = chainRuntime.solver->CartToJnt(seed, perturbed, output);
            }
        }
        if (rc < 0) {
            return false;
        }

        for (size_t i = 0; i < chainRuntime.jointNames.size(); ++i) {
            scene->setJointPositionByName(chainRuntime.jointNames[i], static_cast<float>(output(i)));
        }

        scene->updateTransforms();

        if (outPositionErrorMm != nullptr) {
            glm::vec3 tipPos(0.0f);
            glm::vec3 tipRpy(0.0f);
            if (fetchTipWorldPose(*scene, chainIndex, &tipPos, &tipRpy)) {
                const glm::vec3 targetPos = glm::vec3(targetWorld[3]);
                *outPositionErrorMm       = glm::length(tipPos - targetPos) * 1000.0f;
            } else {
                *outPositionErrorMm = 0.0f;
            }
        }

        return true;
    }

    bool IkSolver::solveSingleChain(RobotScene* scene, int chainIndex, const glm::mat4& targetWorld, std::string* statusText) {
        if (chainIndex < 0 || chainIndex >= chainCount()) {
            if (statusText) {
                *statusText = "IK失败：未选择链";
            }
            return false;
        }

        const auto& chainRuntime = chains_[chainIndex];
        if (!chainRuntime.status.ready || chainRuntime.solver == nullptr) {
            if (statusText) {
                *statusText = "IK失败：链未就绪";
            }
            return false;
        }

        if (!solveChainInternal(scene, chainIndex, chainRuntime, targetWorld, 8, nullptr)) {
            if (statusText) {
                *statusText = "IK失败：不可达/超时（已做扰动重试）";
            }
            return false;
        }

        if (statusText) {
            *statusText = "IK成功";
        }
        return true;
    }

    bool IkSolver::querySingleChain(int chainIndex, const glm::mat4& targetWorld, const glm::mat4& baseWorld,
                                    const std::vector<float>& seedJointPositions, std::vector<float>* outSolution,
                                    std::string* statusText) const {
        if (chainIndex < 0 || chainIndex >= chainCount()) {
            if (statusText) {
                *statusText = "IK失败：未选择链";
            }
            return false;
        }

        const auto& chainRuntime = chains_[chainIndex];
        if (!chainRuntime.status.ready || chainRuntime.solver == nullptr) {
            if (statusText) {
                *statusText = "IK失败：链未就绪";
            }
            return false;
        }

        // Convert target to base frame
        const glm::mat4 targetInBase = glm::inverse(baseWorld) * targetWorld;
        const KDL::Frame target      = glmToKdlFrame(targetInBase);

        // Build seed from provided positions or use neutral pose
        const size_t jointCount = chainRuntime.jointNames.size();
        KDL::JntArray seed(jointCount);
        if (seedJointPositions.size() == jointCount) {
            for (size_t i = 0; i < jointCount; ++i) {
                seed(i) = std::min(chainRuntime.upper(i), std::max(chainRuntime.lower(i), static_cast<double>(seedJointPositions[i])));
            }
        } else {
            for (size_t i = 0; i < jointCount; ++i) {
                seed(i) = 0.5 * (chainRuntime.lower(i) + chainRuntime.upper(i));
            }
        }

        KDL::JntArray output(jointCount);
        int rc = chainRuntime.solver->CartToJnt(seed, target, output);
        if (rc < 0) {
            // Retry with perturbed target
            for (int i = 0; i < 8; ++i) {
                KDL::Frame perturbed = target;
                const double dp      = 0.004 * static_cast<double>(i + 1);
                perturbed.p.x(target.p.x() + dp);
                perturbed.p.y(target.p.y() - dp);
                rc = chainRuntime.solver->CartToJnt(seed, perturbed, output);
                if (rc >= 0) {
                    break;
                }
            }
        }
        if (rc < 0) {
            if (statusText) {
                *statusText = "IK失败：不可达/超时（已做扰动重试）";
            }
            return false;
        }

        if (outSolution != nullptr) {
            outSolution->resize(jointCount);
            for (size_t i = 0; i < jointCount; ++i) {
                (*outSolution)[i] = static_cast<float>(output(i));
            }
        }

        if (statusText) {
            *statusText = "IK成功";
        }
        return true;
    }

    bool IkSolver::initializeFullBodySolvers(const std::string& urdfPath, const std::vector<ViewerIkChainConfig>& chains) {
        fullBodyReady_     = false;
        fullBodyFlexReady_ = false;
        fullBodyWbcReady_  = false;
        fullBodyError_.clear();
        fullBodySolverPose_.reset();
        fullBodySolverPosOnly_.reset();
        fullBodyWbcSolverPose_.reset();
        fullBodyWbcSolverPosOnly_.reset();
        fullBodyJointQIndex_.clear();
        fullBodyLockedVIndex_.clear();
        fullBodyWbcJointQIndex_.clear();
        fullBodyWbcLockedVIndex_.clear();
        fullBodyWbcPlanarBaseVIndex_.clear();
        fullBodyWbcNv_              = 0;
        fullBodyWbcHasFloatingBase_ = false;

        try {
            fullBodySolverPose_    = std::make_unique<flex_ik::FlexIk>(urdfPath);
            fullBodySolverPosOnly_ = std::make_unique<flex_ik::FlexIk>(urdfPath);
        } catch (const std::exception& ex) {
            fullBodyError_ = std::string("flex_ik 初始化失败: ") + ex.what();
            return false;
        }

        for (const auto& chain : chains) {
            if (!fullBodySolverPose_->model().existFrame(chain.tip_link)) {
                fullBodyError_ = "flex_ik 任务帧不存在: " + chain.tip_link;
                return false;
            }
            try {
                fullBodySolverPose_->addTask(chain.tip_link, 1.0);
                flex_ik::Vector6 posMask;
                posMask << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0;
                fullBodySolverPosOnly_->addTask(chain.tip_link, posMask, 1.0);
            } catch (const std::exception& ex) {
                fullBodyError_ = std::string("flex_ik 添加任务失败: ") + ex.what();
                return false;
            }
        }

        const auto& model = fullBodySolverPose_->model();
        std::vector<std::string> fixedJointNames;
        fixedJointNames.reserve(static_cast<size_t>(model.njoints));
        for (int j = 1; j < model.njoints; ++j) {
            const std::string& jointName     = model.names[j];
            const auto& joint                = model.joints[j];
            const bool isScalarActuatedJoint = (joint.nq() == 1 && joint.nv() == 1);
            if (!isScalarActuatedJoint) {
                fixedJointNames.push_back(jointName);
                const int idxV = joint.idx_v();
                for (int k = 0; k < joint.nv(); ++k) {
                    const int vidx = idxV + k;
                    if (vidx >= 0 && vidx < model.nv) {
                        fullBodyLockedVIndex_.push_back(vidx);
                    }
                }
                continue;
            }
            const int idxQ = joint.idx_q();
            if (idxQ < 0 || idxQ >= model.nq) {
                continue;
            }
            fullBodyJointQIndex_[jointName] = idxQ;
        }

        try {
            fullBodySolverPose_->setFixedJoints(fixedJointNames);
            fullBodySolverPosOnly_->setFixedJoints(fixedJointNames);
        } catch (const std::exception& ex) {
            fullBodyError_ = std::string("flex_ik 固定关节配置失败: ") + ex.what();
            return false;
        }

        flex_ik::FlexIk::IterParams poseParams = fullBodySolverPose_->iterativeParams();
        poseParams.max_iterations              = 80;
        poseParams.error_tolerance             = 1e-3;
        poseParams.dq_gain                     = 1.0;
        poseParams.orientation_gain            = 1.0;
        fullBodySolverPose_->setIterativeParams(poseParams);

        flex_ik::FlexIk::IterParams posParams = fullBodySolverPosOnly_->iterativeParams();
        posParams.max_iterations              = 50;
        posParams.error_tolerance             = 2e-3;
        posParams.dq_gain                     = 1.0;
        posParams.orientation_gain            = 1.0;
        fullBodySolverPosOnly_->setIterativeParams(posParams);

        fullBodyFlexReady_ = true;

        try {
            const pinocchio::Model wbcModel = BuildFloatingBaseModelForWbc(urdfPath, &fullBodyWbcHasFloatingBase_);
            fullBodyWbcNv_                  = wbcModel.nv;
            for (int j = 1; j < wbcModel.njoints; ++j) {
                const std::string& jointName     = wbcModel.names[j];
                const auto& joint                = wbcModel.joints[j];
                const bool isScalarActuatedJoint = (joint.nq() == 1 && joint.nv() == 1);
                if (!isScalarActuatedJoint) {
                    const int idxV = joint.idx_v();
                    for (int k = 0; k < joint.nv(); ++k) {
                        const int vidx = idxV + k;
                        if (vidx >= 0 && vidx < wbcModel.nv) {
                            fullBodyWbcLockedVIndex_.push_back(vidx);
                        }
                    }
                    continue;
                }

                const int idxQ = joint.idx_q();
                if (idxQ >= 0 && idxQ < wbcModel.nq) {
                    fullBodyWbcJointQIndex_[jointName] = idxQ;
                }
            }

            // Free-flyer velocity order: [vx, vy, vz, wx, wy, wz]; unlock planar chassis DOFs when assisting.
            if (fullBodyWbcHasFloatingBase_ && wbcModel.nv >= 6) {
                fullBodyWbcPlanarBaseVIndex_ = {0, 1, 5};
            }

            std::vector<Wbc::Index> fixedJointV;
            fixedJointV.reserve(fullBodyWbcLockedVIndex_.size());
            for (const int idx : fullBodyWbcLockedVIndex_) {
                fixedJointV.push_back(static_cast<Wbc::Index>(idx));
            }

            fullBodyWbcSolverPose_    = std::make_unique<Wbc::robot::ChainIkTrait>(wbcModel, fixedJointV);
            fullBodyWbcSolverPosOnly_ = std::make_unique<Wbc::robot::ChainIkTrait>(wbcModel, fixedJointV);
            for (const auto& chain : chains) {
                std::array<bool, 6> poseMask = {true, true, true, true, true, true};
                std::array<bool, 6> posMask  = {true, true, true, false, false, false};
                fullBodyWbcSolverPose_->addIkTask(chain.tip_link, Wbc::SE3::Identity(), 1.0, poseMask);
                fullBodyWbcSolverPosOnly_->addIkTask(chain.tip_link, Wbc::SE3::Identity(), 1.0, posMask);
            }
            fullBodyWbcSolverPose_->setMaxIters(100);
            fullBodyWbcSolverPose_->setTolerance(1e-4);
            fullBodyWbcSolverPose_->setJointWeights(0.6);
            fullBodyWbcSolverPosOnly_->setMaxIters(80);
            fullBodyWbcSolverPosOnly_->setTolerance(2e-4);
            fullBodyWbcSolverPosOnly_->setJointWeights(0.8);
            fullBodyWbcReady_ = true;
        } catch (const std::exception& ex) {
            fullBodyWbcReady_ = false;
            fullBodyError_ += (fullBodyError_.empty() ? "" : "; ");
            fullBodyError_ += std::string("wbc_chain_ik 初始化失败: ") + ex.what();
        }

        fullBodyReady_ = true;
        return true;
    }

    flex_ik::Vector IkSolver::buildFullBodyQFromScene(const RobotScene& scene) const {
        const auto& model = fullBodySolverPose_->model();
        flex_ik::Vector q = pinocchio::neutral(model);
        const auto joints = scene.getJointInfos();
        for (const auto& joint : joints) {
            auto it = fullBodyJointQIndex_.find(joint.name);
            if (it == fullBodyJointQIndex_.end()) {
                continue;
            }
            const int idxQ     = it->second;
            const double lower = model.lowerPositionLimit[idxQ];
            const double upper = model.upperPositionLimit[idxQ];
            q[idxQ]            = std::max(lower, std::min(upper, static_cast<double>(joint.position)));
        }
        return q;
    }

    void IkSolver::applyFullBodyQToScene(RobotScene* scene, const flex_ik::Vector& q) const {
        if (scene == nullptr) {
            return;
        }
        for (const auto& kv : fullBodyJointQIndex_) {
            const int idxQ = kv.second;
            if (idxQ < 0 || idxQ >= q.size()) {
                continue;
            }
            RobotScene::JointInfo jointInfo;
            const bool hasJointInfo = scene->getJointInfo(kv.first, &jointInfo);
            float value             = static_cast<float>(q[idxQ]);
            if (hasJointInfo && jointInfo.revolute) {
                value = std::clamp(value, jointInfo.min_angle, jointInfo.max_angle);
            }
            scene->setJointPositionByName(kv.first, value);
        }
        scene->updateTransforms();
    }

    flex_ik::Vector IkSolver::buildWbcFullBodyQFromScene(const RobotScene& scene) const {
        if (fullBodyWbcSolverPose_ == nullptr) {
            return flex_ik::Vector();
        }
        const int nq      = fullBodyWbcSolverPose_->qMin().size();
        flex_ik::Vector q = flex_ik::Vector::Zero(nq);
        if (fullBodyWbcHasFloatingBase_ && q.size() >= 7) {
            virtualBase2DToFloatingBaseQ(0.0f, 0.0f, 0.0f, &q);
        } else if (fullBodyWbcSolverPose_->qMax().size() == nq) {
            q = 0.5 * (fullBodyWbcSolverPose_->qMin() + fullBodyWbcSolverPose_->qMax());
        }

        const auto joints = scene.getJointInfos();
        for (const auto& joint : joints) {
            auto it = fullBodyWbcJointQIndex_.find(joint.name);
            if (it == fullBodyWbcJointQIndex_.end()) {
                continue;
            }
            const int idxQ = it->second;
            if (idxQ < 0 || idxQ >= q.size()) {
                continue;
            }
            q[idxQ] = static_cast<double>(joint.position);
        }

        if (fullBodyWbcHasFloatingBase_ && q.size() >= 7) {
            float baseX   = 0.0f;
            float baseY   = 0.0f;
            float baseYaw = 0.0f;
            if (!scene.fixedBaseMode() && scene.getVirtualBasePose2D(&baseX, &baseY, &baseYaw)) {
                virtualBase2DToFloatingBaseQ(baseX, baseY, baseYaw, &q);
            } else {
                virtualBase2DToFloatingBaseQ(0.0f, 0.0f, 0.0f, &q);
            }
        }
        return q;
    }

    void IkSolver::applyWbcFullBodyQToScene(RobotScene* scene, const flex_ik::Vector& q) const {
        if (scene == nullptr) {
            return;
        }
        const bool lockBase = scene->fixedBaseMode();
        for (const auto& kv : fullBodyWbcJointQIndex_) {
            if (lockBase && IsBaseMotionJointName(kv.first)) {
                continue;
            }
            const int idxQ = kv.second;
            if (idxQ < 0 || idxQ >= q.size()) {
                continue;
            }
            RobotScene::JointInfo jointInfo;
            const bool hasJointInfo = scene->getJointInfo(kv.first, &jointInfo);
            float value             = static_cast<float>(q[idxQ]);
            if (hasJointInfo && jointInfo.revolute) {
                value = std::clamp(value, jointInfo.min_angle, jointInfo.max_angle);
            }
            scene->setJointPositionByName(kv.first, value);
        }

        if (fullBodyWbcHasFloatingBase_ && q.size() >= 7 && !lockBase) {
            float baseX   = 0.0f;
            float baseY   = 0.0f;
            float baseYaw = 0.0f;
            if (floatingBaseQToVirtualBase2D(q, &baseX, &baseY, &baseYaw)) {
                scene->setVirtualBasePose2D(baseX, baseY, baseYaw);
            }
        }
        scene->updateTransforms();
    }

    flex_ik::Vector IkSolver::limitFullBodyStep(const flex_ik::Vector& qCurrent, const flex_ik::Vector& qSolved, bool fastMode,
                                                bool positionOnlyMode) const {
        flex_ik::Vector qOut = qCurrent;
        if (qCurrent.size() != qSolved.size()) {
            return qOut;
        }
        if (fullBodySolverPose_ == nullptr) {
            return qOut;
        }

        const auto& model     = fullBodySolverPose_->model();
        const double maxDelta = fastMode ? (positionOnlyMode ? 0.25 : 0.18) : (positionOnlyMode ? 1.2 : 0.65);
        for (const auto& kv : fullBodyJointQIndex_) {
            const int idxQ = kv.second;
            if (idxQ < 0 || idxQ >= qSolved.size() || idxQ >= qCurrent.size()) {
                continue;
            }

            const double target  = qSolved[idxQ];
            const double current = qCurrent[idxQ];
            if (!std::isfinite(target) || !std::isfinite(current)) {
                continue;
            }

            double lower = model.lowerPositionLimit[idxQ];
            double upper = model.upperPositionLimit[idxQ];
            if (!std::isfinite(lower) || lower < -1e6) {
                lower = -3.14159265358979323846;
            }
            if (!std::isfinite(upper) || upper > 1e6) {
                upper = 3.14159265358979323846;
            }
            if (lower > upper) {
                std::swap(lower, upper);
            }

            const double boundedTarget = std::clamp(target, lower, upper);
            const double delta         = std::clamp(boundedTarget - current, -maxDelta, maxDelta);
            qOut[idxQ]                 = current + delta;
        }
        return qOut;
    }

    flex_ik::Vector IkSolver::limitWbcFullBodyStep(const flex_ik::Vector& qCurrent, const flex_ik::Vector& qSolved, bool fastMode,
                                                   bool positionOnlyMode, bool lockFloatingBase) const {
        flex_ik::Vector qOut = qCurrent;
        if (qCurrent.size() != qSolved.size()) {
            return qOut;
        }

        const double jointMaxDelta = fastMode ? (positionOnlyMode ? 0.25 : 0.18) : (positionOnlyMode ? 1.2 : 0.65);
        for (const auto& kv : fullBodyWbcJointQIndex_) {
            const int idxQ = kv.second;
            if (idxQ < 0 || idxQ >= qSolved.size() || idxQ >= qCurrent.size()) {
                continue;
            }
            const double target  = qSolved[idxQ];
            const double current = qCurrent[idxQ];
            if (!std::isfinite(target) || !std::isfinite(current)) {
                continue;
            }
            const double delta = std::clamp(target - current, -jointMaxDelta, jointMaxDelta);
            qOut[idxQ]         = current + delta;
        }

        if (!lockFloatingBase && fullBodyWbcHasFloatingBase_ && qCurrent.size() >= 7 && qSolved.size() >= 7) {
            // Allow larger chassis steps on drag-end solve; realtime drag stays conservative.
            const double basePosDelta = fastMode ? 0.08 : 0.20;
            const double baseYawDelta = fastMode ? 0.12 : 0.30;
            float curX                = 0.0f;
            float curY                = 0.0f;
            float curYaw              = 0.0f;
            float tgtX                = 0.0f;
            float tgtY                = 0.0f;
            float tgtYaw              = 0.0f;
            if (floatingBaseQToVirtualBase2D(qCurrent, &curX, &curY, &curYaw) &&
                floatingBaseQToVirtualBase2D(qSolved, &tgtX, &tgtY, &tgtYaw)) {
                const float x   = curX + std::clamp(tgtX - curX, static_cast<float>(-basePosDelta), static_cast<float>(basePosDelta));
                const float y   = curY + std::clamp(tgtY - curY, static_cast<float>(-basePosDelta), static_cast<float>(basePosDelta));
                const float yaw = curYaw + std::clamp(tgtYaw - curYaw, static_cast<float>(-baseYawDelta), static_cast<float>(baseYawDelta));
                virtualBase2DToFloatingBaseQ(x, y, yaw, &qOut);
            }
        }

        return qOut;
    }

    bool IkSolver::solveFullBody(RobotScene* scene, const std::vector<glm::mat4>& targetWorldByChain, int iterations, int activeChainIndex,
                                 bool fastMode, bool positionOnlyMode, IkSolveStats* stats, std::string* statusText) {
        if (scene == nullptr) {
            if (statusText) {
                *statusText = "IK失败：场景为空";
            }
            return false;
        }
        if (targetWorldByChain.size() != chains_.size()) {
            if (statusText) {
                *statusText = "IK失败：目标数量与链数量不匹配";
            }
            return false;
        }
        if (!fullBodyReady_) {
            if (statusText) {
                *statusText = "IK失败：flex_ik未就绪" + (fullBodyError_.empty() ? std::string() : (" (" + fullBodyError_ + ")"));
            }
            return false;
        }

        int readyCount = 0;
        for (int i = 0; i < chainCount(); ++i) {
            if (chains_[i].status.ready) {
                ++readyCount;
            }
        }
        if (readyCount <= 0) {
            if (statusText) {
                *statusText = "IK失败：无可用链";
            }
            return false;
        }

        int solvedCount   = 0;
        bool activeSolved = false;
        float maxErrorMm  = 0.0f;
        bool success      = false;
        bool usedFallback = false;

        if (fullBodyBackend_ == "wbc_chain_ik") {
            if (!fullBodyWbcReady_ || fullBodyWbcSolverPose_ == nullptr || fullBodyWbcSolverPosOnly_ == nullptr) {
                if (statusText) {
                    *statusText = "IK失败：wbc_chain_ik未就绪" + (fullBodyError_.empty() ? std::string() : (" (" + fullBodyError_ + ")"));
                }
                return false;
            }
            const flex_ik::Vector qCurrent = buildWbcFullBodyQFromScene(*scene);

            Wbc::robot::ChainIkTrait* wbcSolver = positionOnlyMode ? fullBodyWbcSolverPosOnly_.get() : fullBodyWbcSolverPose_.get();
            // Realtime drag must not use tiny iteration budgets (iterations=1 from UI would freeze motion).
            wbcSolver->setMaxIters(fastMode ? 80 : std::max(80, iterations * 25));
            wbcSolver->setTolerance(fastMode ? 2e-3 : 1e-4);

            const bool allowMobileBasePlanar =
                fullBodyWbcHasFloatingBase_ && !fullBodyWbcPlanarBaseVIndex_.empty() && !scene->fixedBaseMode();
            const bool lockPlanarBase = !allowMobileBasePlanar;
            const std::vector<Wbc::Index> fixedJointV =
                buildWbcFixedVelocityIndices(fullBodyWbcLockedVIndex_, fullBodyWbcPlanarBaseVIndex_, lockPlanarBase);
            wbcSolver->setFixedJoints(fixedJointV);

            // Unified solve (wbc_ik_impl): unlock vx/vy/wz when mobile base assists; lower weight on those DOFs.
            constexpr double kDefaultJointWeight = 1e-2;
            constexpr double kPlanarBaseWeight   = 5e-3;
            if (fullBodyWbcNv_ > 0) {
                Wbc::Vector weights = Wbc::Vector::Constant(fullBodyWbcNv_, kDefaultJointWeight);
                if (allowMobileBasePlanar) {
                    for (const int idx : fullBodyWbcPlanarBaseVIndex_) {
                        if (idx >= 0 && idx < weights.size()) {
                            weights(idx) = kPlanarBaseWeight;
                        }
                    }
                }
                wbcSolver->setJointWeights(weights);
            } else {
                wbcSolver->setJointWeights(kDefaultJointWeight);
            }

            for (int i = 0; i < chainCount(); ++i) {
                if (!chains_[i].status.ready) {
                    continue;
                }
                try {
                    wbcSolver->setTaskReference(chains_[i].status.config.tip_link,
                                                glmToFlexSe3(targetWorldByChain[static_cast<size_t>(i)]));
                    const double taskWeight = (i == activeChainIndex)
                                                  ? (fastMode ? (positionOnlyMode ? 8.0 : 12.0) : (positionOnlyMode ? 12.0 : 16.0))
                                                  : (positionOnlyMode ? 0.02 : 0.03);
                    wbcSolver->setTaskWeight(chains_[i].status.config.tip_link, taskWeight);
                } catch (const std::exception& ex) {
                    if (statusText) {
                        *statusText = std::string("IK失败：wbc_chain_ik更新目标失败: ") + ex.what();
                    }
                    return false;
                }
            }

            Wbc::robot::IkResult wbcResult = wbcSolver->solveIK(qCurrent, false);

            if (wbcResult.solution.size() == 0) {
                if (statusText) {
                    *statusText = "IK失败：wbc_chain_ik未返回解";
                }
                return false;
            }

            const bool lockFloatingBase = scene->fixedBaseMode();
            flex_ik::Vector qApply      = fastMode
                                              ? wbcResult.solution
                                              : limitWbcFullBodyStep(qCurrent, wbcResult.solution, false, positionOnlyMode, lockFloatingBase);
            if (lockFloatingBase && fullBodyWbcHasFloatingBase_ && qApply.size() >= 7) {
                virtualBase2DToFloatingBaseQ(0.0f, 0.0f, 0.0f, &qApply);
            }
            // Realtime drag: apply solver output directly (internal dq limits already cap per-iter motion).
            applyWbcFullBodyQToScene(scene, qApply);

            if (!wbcResult.success && !fastMode && activeChainIndex >= 0 && activeChainIndex < chainCount()) {
                glm::vec3 tipPos(0.0f);
                glm::vec3 tipRpy(0.0f);
                if (fetchTipWorldPose(*scene, activeChainIndex, &tipPos, &tipRpy)) {
                    const glm::vec3 targetPos = glm::vec3(targetWorldByChain[static_cast<size_t>(activeChainIndex)][3]);
                    const float activeErrMm   = glm::length(tipPos - targetPos) * 1000.0f;
                    if (activeErrMm > 200.0f) {
                        applyWbcFullBodyQToScene(scene, qCurrent);
                        if (statusText) {
                            if (scene->fixedBaseMode()) {
                                *statusText = "IK失败：未收敛；已开启「固定底座模式」，底盘不会参与（请取消勾选）";
                            } else {
                                *statusText = "IK失败：wbc_chain_ik未收敛且偏差过大，已回滚";
                            }
                        }
                        return false;
                    }
                    if (statusText) {
                        *statusText = "IK未完全收敛，已应用当前最优解";
                    }
                }
            }
            success = wbcResult.success || fastMode;
        } else {
            if (!fullBodyFlexReady_ || fullBodySolverPose_ == nullptr || fullBodySolverPosOnly_ == nullptr) {
                if (statusText) {
                    *statusText = "IK失败：flex_ik未就绪" + (fullBodyError_.empty() ? std::string() : (" (" + fullBodyError_ + ")"));
                }
                return false;
            }
            const flex_ik::Vector qCurrent = buildFullBodyQFromScene(*scene);
            auto applyCandidate            = [&](const flex_ik::Vector& solution, bool solverSuccess, const std::string& solverName) {
                applyFullBodyQToScene(scene, limitFullBodyStep(qCurrent, solution, fastMode, positionOnlyMode));
                if (!solverSuccess && !fastMode && !positionOnlyMode && activeChainIndex >= 0 && activeChainIndex < chainCount()) {
                    glm::vec3 tipPos(0.0f);
                    glm::vec3 tipRpy(0.0f);
                    if (fetchTipWorldPose(*scene, activeChainIndex, &tipPos, &tipRpy)) {
                        const glm::vec3 targetPos = glm::vec3(targetWorldByChain[static_cast<size_t>(activeChainIndex)][3]);
                        const float activeErrMm   = glm::length(tipPos - targetPos) * 1000.0f;
                        if (activeErrMm > 120.0f) {
                            applyFullBodyQToScene(scene, qCurrent);
                            if (statusText) {
                                *statusText = "IK失败：" + solverName + "姿态求解未收敛且位置偏差过大，保持当前姿态";
                            }
                            return false;
                        }
                    }
                }
                return true;
            };

            flex_ik::FlexIk* solver = positionOnlyMode ? fullBodySolverPosOnly_.get() : fullBodySolverPose_.get();
            auto params             = solver->iterativeParams();
            params.max_iterations   = fastMode ? std::max(6, iterations * (positionOnlyMode ? 4 : 8)) : std::max(20, iterations * 15);
            params.error_tolerance  = fastMode ? (positionOnlyMode ? 6e-3 : 3e-3) : 1e-3;
            params.dq_gain          = fastMode ? 0.8 : 1.0;
            params.orientation_gain = positionOnlyMode ? 1.0 : (fastMode ? 0.6 : 1.0);
            solver->setIterativeParams(params);

            for (int i = 0; i < chainCount(); ++i) {
                if (!chains_[i].status.ready) {
                    continue;
                }
                try {
                    solver->updateTaskTarget(chains_[i].status.config.tip_link, glmToFlexSe3(targetWorldByChain[static_cast<size_t>(i)]));
                    const double taskWeight = (i == activeChainIndex)
                                                  ? (fastMode ? (positionOnlyMode ? 8.0 : 12.0) : (positionOnlyMode ? 12.0 : 16.0))
                                                  : (positionOnlyMode ? 0.02 : 0.03);
                    solver->setTaskWeight(chains_[i].status.config.tip_link, taskWeight);
                } catch (const std::exception& ex) {
                    if (statusText) {
                        *statusText = std::string("IK失败：flex_ik更新目标失败: ") + ex.what();
                    }
                    return false;
                }
            }

            flex_ik::Vector q0               = qCurrent;
            flex_ik::FlexIk::IkResult result = solver->solve(q0);
            if (!result.success && !fastMode && positionOnlyMode) {
                auto fallbackParams             = fullBodySolverPosOnly_->iterativeParams();
                fallbackParams.max_iterations   = std::max(20, iterations * 15);
                fallbackParams.error_tolerance  = 2e-3;
                fallbackParams.dq_gain          = 1.0;
                fallbackParams.orientation_gain = 1.0;
                fullBodySolverPosOnly_->setIterativeParams(fallbackParams);
                for (int i = 0; i < chainCount(); ++i) {
                    if (!chains_[i].status.ready) {
                        continue;
                    }
                    fullBodySolverPosOnly_->updateTaskTarget(chains_[i].status.config.tip_link,
                                                             glmToFlexSe3(targetWorldByChain[static_cast<size_t>(i)]));
                    const double taskWeight = (i == activeChainIndex) ? 12.0 : 0.02;
                    fullBodySolverPosOnly_->setTaskWeight(chains_[i].status.config.tip_link, taskWeight);
                }
                result       = fullBodySolverPosOnly_->solve(q0);
                usedFallback = true;
            }

            if (result.solution.size() == 0) {
                if (statusText) {
                    *statusText = "IK失败：flex_ik未返回解";
                }
                return false;
            }
            if (!applyCandidate(result.solution, result.success, "flex_ik")) {
                return false;
            }
            success = result.success;
        }

        const float successPosErrMm = fastMode ? 45.0f : 25.0f;

        for (int i = 0; i < chainCount(); ++i) {
            if (!chains_[i].status.ready) {
                continue;
            }
            glm::vec3 tipPos(0.0f);
            glm::vec3 tipRpy(0.0f);
            if (!fetchTipWorldPose(*scene, i, &tipPos, &tipRpy)) {
                continue;
            }
            const glm::vec3 targetPos = glm::vec3(targetWorldByChain[static_cast<size_t>(i)][3]);
            const float posErrMm      = glm::length(tipPos - targetPos) * 1000.0f;
            maxErrorMm                = std::max(maxErrorMm, posErrMm);
            if (posErrMm <= successPosErrMm) {
                ++solvedCount;
                if (i == activeChainIndex) {
                    activeSolved = true;
                }
            }
        }

        if (activeChainIndex >= 0 && activeChainIndex < chainCount()) {
            success = success || activeSolved;
        }

        if (stats != nullptr) {
            stats->success               = success;
            stats->solved_chain_count    = solvedCount;
            stats->ready_chain_count     = std::max(readyCount, 0);
            stats->active_chain_solved   = activeSolved;
            stats->max_position_error_mm = maxErrorMm;
        }

        if (statusText) {
            std::ostringstream oss;
            const std::string solverLabel = (fullBodyBackend_ == "wbc_chain_ik") ? "WBC-ChainIK" : "FlexIK-QP";
            if (success) {
                oss << "全身IK成功(" << solverLabel << "): 活动链=" << (activeSolved ? "ok" : "fail") << ", 总体 " << solvedCount << "/"
                    << std::max(readyCount, 0) << " 链接近目标";
                if (maxErrorMm > 0.0f) {
                    oss << "，最大误差 " << maxErrorMm << " mm";
                }
                if (usedFallback) {
                    oss << "（fallback: position-only）";
                }
            } else {
                oss << "全身IK失败(" << solverLabel << "): 活动链未到位, 总体 " << solvedCount << "/" << std::max(readyCount, 0)
                    << " 链接近目标";
                if (usedFallback) {
                    oss << "（position-only仍失败）";
                }
            }
            *statusText = oss.str();
        }
        return success;
    }

}  // namespace rkv
