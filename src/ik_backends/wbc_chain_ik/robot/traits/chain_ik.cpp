#include "wbc/robot/traits/chain_ik.h"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "wbc/math/helper_functions.h"
namespace Wbc {
    namespace robot {

        ChainIkTrait::ChainIkTrait(const pinocchio::Model& model, const std::vector<Index>& fixed_joints)
            : model_(model), data_(model), nv(model.nv), fixed_joints(fixed_joints), hqp_solver() {
            hqp_data.reset(nv, 0, nv);
            J.setZero(6, nv);
            q_min         = model.lowerPositionLimit;
            q_max         = model.upperPositionLimit;
            joint_weights = Vector::Constant(nv, 1e-2);

            J_com.setZero(3, nv);
            const Vector q_neutral = pinocchio::neutral(model_);
            pinocchio::centerOfMass(model_, data_, q_neutral);
            com_ref = data_.com[0];
            com_weight.setConstant(1.0);
        }

        void ChainIkTrait::addIkTask(const std::string& frame_name, const SE3& target_pose, double weight,
                                     const std::array<bool, 6>& mask) {
            tasks[frame_name] = IkConfig{model_.getFrameId(frame_name), target_pose, weight, std::make_optional(mask)};
            hqp_data.reset(nv, 6 * tasks.size(), nv);
        }

        IkResult ChainIkTrait::solveIK(const Vector& q_current, bool verbose) {
            auto time_start = std::chrono::high_resolution_clock::now();
            IkResult result;

            int count      = 1;
            Vector q_out   = q_current;
            bool converged = false;
            double err_sum = 0.0;
            Vector6 p_error;
            Vector delta_q(nv);

            while (count < max_iters) {
                hqp_data.reset(nv, 6 * tasks.size(), nv);  // set to zero
                auto& qp0 = hqp_data.qp0;
                auto& J0  = hqp_data.J0;
                auto& qp1 = hqp_data.qp1;
                err_sum   = 0.0;
                pinocchio::computeJointJacobians(model_, data_, q_out);
                pinocchio::updateFramePlacements(model_, data_);

                // int row_idx = 0;
                // for (const auto& [frame_name, config] : tasks) {
                //     p_error = math::errorInSE3(data_.oMf[config.frame_id], config.target_pose).toVector();
                //     if (config.mask.has_value()) {
                //         for (int i = 0; i < 6; i++) {
                //             if (!config.mask->at(i)) {
                //                 p_error(i) = 0.0;
                //                 J.row(i).setZero();
                //             }
                //         }
                //     }
                //     qp0.H += config.weight * (J.transpose() * J);
                //     qp0.g += -config.weight * (J.transpose() * p_error);
                //     J0.middleRows(row_idx, 6) = J;
                //     row_idx += 6;
                //     err_sum += p_error.squaredNorm();
                // }

                int row_idx = 0;

                for (const auto& [frame_name, config] : tasks) {
                    // 当前末端位姿：world -> frame
                    // const pinocchio::SE3& oMf = data_.oMf[config.frame_id];

                    // // 目标末端位姿：world -> desired frame
                    // const pinocchio::SE3& oMdes = config.target_pose;

                    // // 当前 frame 到目标 frame 的误差：current^-1 * desired
                    // pinocchio::SE3 iMd = oMf.actInv(oMdes);

                    // // Pinocchio 标准 SE(3) log 误差，LOCAL 表达
                    // Vector6 err = pinocchio::log6(iMd).toVector();

                    // // 强烈建议加误差限幅，避免一次旋转目标太大
                    // const double max_pos_err = 0.03;  // meter
                    // const double max_rot_err = 0.05;  // rad

                    // const double pos_norm = err.head<3>().norm();
                    // if (pos_norm > max_pos_err) {
                    //     err.head<3>() *= max_pos_err / pos_norm;
                    // }

                    // const double rot_norm = err.tail<3>().norm();
                    // if (rot_norm > max_rot_err) {
                    //     err.tail<3>() *= max_rot_err / rot_norm;
                    // }

                    // // 重新计算当前 frame 的 LOCAL Jacobian
                    // J.setZero();
                    // pinocchio::getFrameJacobian(model_, data_, config.frame_id, pinocchio::LOCAL, J);

                    // // Jlog6 修正
                    // Eigen::Matrix<double, 6, 6> Jlog;
                    // pinocchio::Jlog6(iMd.inverse(), Jlog);

                    // // Pinocchio 标准 IK convention
                    // Eigen::MatrixXd J_task = -Jlog * J;
                    // Vector6 rhs            = -err;

                    auto lin = math::computeFrameIkLinearizationLocal(model_, data_, config.frame_id, config.target_pose,
                                                                      0.03,  // max_pos_err, meter
                                                                      0.05   // max_rot_err, rad
                    );

                    Eigen::MatrixXd J_task = lin.J;
                    Vector6 rhs            = lin.rhs;
                    Vector6 err            = lin.err;

                    // mask 要作用在 J_task 和 rhs 上，而不是旧的 J/p_error 上
                    if (config.mask.has_value()) {
                        for (int i = 0; i < 6; ++i) {
                            if (!config.mask->at(i)) {
                                rhs(i) = 0.0;
                                J_task.row(i).setZero();
                            }
                        }
                    }

                    qp0.H += config.weight * (J_task.transpose() * J_task);
                    qp0.g += -config.weight * (J_task.transpose() * rhs);

                    J0.middleRows(row_idx, 6) = J_task;
                    row_idx += 6;

                    err_sum += rhs.squaredNorm();
                }

                if (std::sqrt(err_sum) < tol) {
                    result.solution   = q_out;
                    result.success    = true;
                    result.converged  = true;
                    result.time_spent = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - time_start).count();
                    if (verbose) {
                        std::cout << "IK solved in " << count - 1 << " iterations with error " << std::sqrt(err_sum) << " and time "
                                  << result.time_spent * 1000.0 << " ms." << std::endl;
                    }
                    return result;
                } else if (converged) {
                    result.solution   = q_out;
                    result.success    = false;
                    result.converged  = true;
                    result.time_spent = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - time_start).count();
                    if (verbose) {
                        std::cout << "IK converged but not solved in " << count << " iterations with error " << std::sqrt(err_sum)
                                  << " and time " << result.time_spent * 1000.0 << " ms." << std::endl;
                    }
                    return result;
                }

                const double dq_step_limit = (model_.nq == model_.nv) ? 0.02 : 0.05;
                qp0.CI.setIdentity();
                if (model_.nq == model_.nv) {
                    // q_min - q_out < dq < q_max - q_out
                    qp0.ci_lb.noalias() = q_min - q_out;
                    qp0.ci_ub.noalias() = q_max - q_out;
                } else {
                    // Floating base: nq != nv; rely on per-step velocity limits in tangent space.
                    qp0.ci_lb.setConstant(-dq_step_limit);
                    qp0.ci_ub.setConstant(dq_step_limit);
                }
                // Zero velocity for fixed joints
                for (const auto& idx : fixed_joints) {
                    qp0.ci_lb(idx) = 0.0;
                    qp0.ci_ub(idx) = 0.0;
                }

                // Per-iteration dq limit (after joint bounds / fixed joints).
                for (int i = 0; i < nv; ++i) {
                    qp0.ci_lb(i) = std::max(qp0.ci_lb(i), -dq_step_limit);
                    qp0.ci_ub(i) = std::min(qp0.ci_ub(i), dq_step_limit);
                }

                // 1. 加 damping：放在所有 task 累加完 H/g 之后
                const double lambda = 5e-2;
                qp0.H.diagonal().array() += lambda;

                // posture regularization
                const double posture_weight = 1e-1;

                for (int i = 0; i < nv; ++i) {
                    qp0.H(i, i) += posture_weight;
                    if (model_.nq == model_.nv) {
                        qp0.g(i) += posture_weight * (q_out(i) - q_current(i));
                    }
                }

                // Secondary objective: minimize joint motion (delta_q)
                for (int i = 0; i < nv; i++) {
                    qp1.H(i, i) += joint_weights(i);
                }

                // addComCost(qp1, q_out);

                const auto& qp_output = hqp_solver.solve(hqp_data);

                if (qp_output.status != solver::QPStatus::OPTIMAL) {
                    result.solution   = q_out;
                    result.success    = false;
                    result.converged  = false;
                    result.time_spent = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - time_start).count();
                    if (verbose) {
                        std::cout << "IK failed at iteration " << count << " with status " << static_cast<int>(qp_output.status)
                                  << " and time " << result.time_spent * 1000.0 << " ms." << std::endl;
                    }
                    return result;
                }

                delta_q = qp_output.x;

                // q_out += delta_q;
                q_out     = pinocchio::integrate(model_, q_out, delta_q);
                converged = delta_q.lpNorm<Eigen::Infinity>() < 1e-8;

                if (verbose) {
                    std::cout << "Iteration " << count << ": error = " << std::sqrt(err_sum) << ", delta_q norm = " << delta_q.norm()
                              << std::endl;
                }

                count++;
            }
            result.solution   = q_out;
            result.success    = false;
            result.converged  = converged;
            result.time_spent = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - time_start).count();
            if (verbose) {
                std::cout << "IK not solved after " << max_iters << " iterations. Final error: " << std::sqrt(err_sum) << " and time "
                          << result.time_spent * 1000.0 << " ms." << std::endl;
            }
            return result;
        }

        void ChainIkTrait::addComCost(solver::QPData& qp, const Vector& q) {
            pinocchio::centerOfMass(model_, data_, q);
            J_com = pinocchio::jacobianCenterOfMass(model_, data_, q, false);
            qp.H += J_com.transpose() * com_weight.asDiagonal() * J_com;
            qp.g += -(J_com.transpose() * com_weight.asDiagonal() * (com_ref - data_.com[0]));
            // std::cout << "com ref: " << com_ref.transpose() << ", com current: " << data_.com[0].transpose() << std::endl;
        }

    }  // namespace robot
}  // namespace Wbc
