// ################################
// C++: fixed-base 6-DoF arm IK implementation begin
// ################################
#include <mm_config/fixed_base_arm_ik.hpp>

#include <mm_config/ee_kinematics_utils.hpp>

#include <algorithm>
#include <cmath>

namespace remani_planner
{
namespace
{

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

bool validParams(const FixedBaseArmIkParams &p)
{
    return std::isfinite(p.pos_tol) && p.pos_tol >= 0.0
        && std::isfinite(p.rot_tol_rad) && p.rot_tol_rad >= 0.0
        && p.max_iters > 0
        && std::isfinite(p.task_rot_weight) && p.task_rot_weight > 0.0
        && std::isfinite(p.joint_weight) && p.joint_weight > 0.0
        && std::isfinite(p.lambda_init) && p.lambda_init > 0.0
        && std::isfinite(p.lambda_min) && p.lambda_min > 0.0
        && std::isfinite(p.lambda_max) && p.lambda_max >= p.lambda_min
        && p.lambda_init >= p.lambda_min && p.lambda_init <= p.lambda_max
        && p.lambda_retry_max > 0
        && std::isfinite(p.step_tol) && p.step_tol >= 0.0
        && p.stagnation_iters > 0;
}

void clampJoints(Eigen::VectorXd &q,
                 const Eigen::VectorXd &q_min,
                 const Eigen::VectorXd &q_max)
{
    for(int i = 0; i < 6; ++i){
        q(i) = std::max(q_min(i), std::min(q_max(i), q(i)));
    }
}

bool computeError(const MMConfig::Ptr &cfg,
                  const Eigen::Vector3d &car_xyyaw,
                  const Eigen::Matrix4d &T_goal,
                  const Eigen::VectorXd &q,
                  Vector6d &error,
                  double &pos_err,
                  double &rot_err)
{
    const Eigen::Matrix4d T_now = cfg->getEePose(car_xyyaw, q);
    if(!T_now.allFinite()){
        return false;
    }

    poseError(T_now, T_goal, error);
    pos_err = error.head<3>().norm();
    rot_err = error.tail<3>().norm();
    return error.allFinite()
        && std::isfinite(pos_err)
        && std::isfinite(rot_err);
}

double weightedErrorSquared(const Vector6d &error, double rot_weight)
{
    Vector6d weighted = error;
    weighted.tail<3>() *= rot_weight;
    return weighted.squaredNorm();
}

} // namespace

FixedBaseArmIk::FixedBaseArmIk(const MMConfig::Ptr &cfg)
    : cfg_(cfg)
{
}

FixedBaseArmIkResult FixedBaseArmIk::solve(
    const Eigen::Vector3d &car_xyyaw,
    const Eigen::Matrix4d &T_goal,
    const Eigen::VectorXd &q_seed,
    const FixedBaseArmIkParams &p,
    const ros::WallTime &deadline)
{
    FixedBaseArmIkResult result;
    result.q = q_seed;

    if(!cfg_ || q_seed.size() != 6
       || cfg_->getManiDof() != 6
       || cfg_->getManipulatorMinPos().size() != 6
       || cfg_->getManipulatorMaxPos().size() != 6
       || !car_xyyaw.allFinite() || !T_goal.allFinite()
       || !q_seed.allFinite() || !validParams(p)){
        result.fail_reason = "invalid_input";
        return result;
    }

    const Eigen::VectorXd &q_min = cfg_->getManipulatorMinPos();
    const Eigen::VectorXd &q_max = cfg_->getManipulatorMaxPos();
    if(!q_min.allFinite() || !q_max.allFinite()
       || (q_min.array() > q_max.array()).any()){
        result.fail_reason = "invalid_joint_limits";
        return result;
    }
    clampJoints(result.q, q_min, q_max);

    Vector6d error;
    if(!computeError(cfg_, car_xyyaw, T_goal, result.q, error,
                     result.pos_err, result.rot_err_rad)){
        result.fail_reason = "non_finite_error";
        return result;
    }

    double lambda = p.lambda_init;
    int stagnation_count = 0;
    const double inverse_weight_squared =
        1.0 / (p.joint_weight * p.joint_weight);

    for(int iter = 0; iter < p.max_iters; ++iter){
        result.iters = iter;
        if(result.pos_err <= p.pos_tol
           && result.rot_err_rad <= p.rot_tol_rad){
            result.success = true;
            result.fail_reason.clear();
            return result;
        }
        if(ros::WallTime::now() >= deadline){
            result.fail_reason = "timeout";
            return result;
        }

        Eigen::Matrix<double, 6, 9> J;
        cfg_->getEeJacobian(car_xyyaw, result.q, J);
        if(!J.allFinite()){
            result.fail_reason = "non_finite_jacobian";
            return result;
        }

        const Matrix6d J_arm = J.block<6, 6>(0, 3);
        Matrix6d S = Matrix6d::Identity();
        S.bottomRightCorner<3, 3>() *= p.task_rot_weight;
        const Matrix6d A_arm = S * J_arm;
        const Vector6d b = S * error;
        const double current_cost = b.squaredNorm();

        bool accepted = false;
        double accepted_step_norm = 0.0;
        for(int retry = 0; retry < p.lambda_retry_max; ++retry){
            if(ros::WallTime::now() >= deadline){
                result.fail_reason = "timeout";
                return result;
            }

            Matrix6d lhs =
                inverse_weight_squared * A_arm * A_arm.transpose();
            lhs.diagonal().array() += lambda * lambda;
            const Eigen::LDLT<Matrix6d> ldlt(lhs);
            if(ldlt.info() != Eigen::Success){
                lambda = std::min(p.lambda_max, lambda * 10.0);
                continue;
            }

            const Vector6d y = ldlt.solve(b);
            if(ldlt.info() != Eigen::Success || !y.allFinite()){
                lambda = std::min(p.lambda_max, lambda * 10.0);
                continue;
            }

            const Vector6d delta_q =
                inverse_weight_squared * A_arm.transpose() * y;
            if(!delta_q.allFinite()){
                lambda = std::min(p.lambda_max, lambda * 10.0);
                continue;
            }

            Eigen::VectorXd q_trial = result.q + delta_q;
            clampJoints(q_trial, q_min, q_max);
            Vector6d trial_error;
            double trial_pos_err = 0.0;
            double trial_rot_err = 0.0;
            if(!computeError(cfg_, car_xyyaw, T_goal, q_trial, trial_error,
                             trial_pos_err, trial_rot_err)){
                lambda = std::min(p.lambda_max, lambda * 10.0);
                continue;
            }

            if(weightedErrorSquared(trial_error, p.task_rot_weight)
               < current_cost){
                accepted_step_norm = (q_trial - result.q).norm();
                result.q = q_trial;
                error = trial_error;
                result.pos_err = trial_pos_err;
                result.rot_err_rad = trial_rot_err;
                lambda = std::max(p.lambda_min, lambda * 0.5);
                accepted = true;
                break;
            }
            lambda = std::min(p.lambda_max, lambda * 10.0);
        }

        result.iters = iter + 1;
        if(!accepted || accepted_step_norm <= p.step_tol){
            ++stagnation_count;
        }else{
            stagnation_count = 0;
        }
        if(stagnation_count >= p.stagnation_iters){
            result.fail_reason = "stagnation";
            return result;
        }
    }

    if(result.pos_err <= p.pos_tol && result.rot_err_rad <= p.rot_tol_rad){
        result.success = true;
        result.fail_reason.clear();
    }else{
        result.fail_reason = "pose_not_reached";
    }
    return result;
}

} // namespace remani_planner
// ################################
// C++: fixed-base 6-DoF arm IK implementation end
// ################################
