// ################################
// C++: whole-body IK solver implementation begin
// ################################
#include <mm_config/whole_body_ik.hpp>
#include <mm_config/ee_kinematics_utils.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace remani_planner
{
namespace
{

constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;

bool isPositiveFinite(double value)
{
    return std::isfinite(value) && value > 0.0;
}

bool isNonNegativeFinite(double value)
{
    return std::isfinite(value) && value >= 0.0;
}

std::vector<double> loadYawOffsetsRad(ros::NodeHandle &nh)
{
    std::vector<double> yaw_offsets_deg;
    nh.param("fsm/ee_ik_c_yaw_offsets_deg",
             yaw_offsets_deg,
             std::vector<double>{0.0, -15.0, 15.0});

    std::vector<double> yaw_offsets_rad;
    yaw_offsets_rad.reserve(yaw_offsets_deg.size());
    for(const double deg : yaw_offsets_deg){
        yaw_offsets_rad.push_back(deg * kDeg2Rad);
    }
    return yaw_offsets_rad;
}

bool validateWholeBodyIkParams(const WholeBodyIkParams &p, std::string &reason)
{
    std::ostringstream oss;

    if(p.ee_goal_topic.empty()){
        oss << "ee_goal_topic empty; ";
    }
    if(p.ee_current_pose_topic.empty()){
        oss << "ee_current_pose_topic empty; ";
    }

    if(!isPositiveFinite(p.ik_pos_tol)){
        oss << "ik_pos_tol invalid; ";
    }
    if(!isPositiveFinite(p.ik_rot_tol_rad)){
        oss << "ik_rot_tol_rad invalid; ";
    }
    if(!isPositiveFinite(p.reach_pos_tol)){
        oss << "reach_pos_tol invalid; ";
    }
    if(!isPositiveFinite(p.reach_rot_tol_rad)){
        oss << "reach_rot_tol_rad invalid; ";
    }
    if(!isPositiveFinite(p.local_xy)){
        oss << "local_xy invalid; ";
    }
    if(!isPositiveFinite(p.local_yaw_rad)){
        oss << "local_yaw_rad invalid; ";
    }
    if(!(p.b_max_iters > 0)){
        oss << "b_max_iters invalid; ";
    }
    if(!isPositiveFinite(p.b_max_ms)){
        oss << "b_max_ms invalid; ";
    }
    if(!isPositiveFinite(p.b_weight_xy)){
        oss << "b_weight_xy invalid; ";
    }
    if(!isPositiveFinite(p.b_weight_yaw)){
        oss << "b_weight_yaw invalid; ";
    }
    if(!isPositiveFinite(p.b_weight_joint)){
        oss << "b_weight_joint invalid; ";
    }
    if(!isNonNegativeFinite(p.b_accept_joint_margin)){
        oss << "b_accept_joint_margin invalid; ";
    }
    if(!isNonNegativeFinite(p.b_accept_clearance)){
        oss << "b_accept_clearance invalid; ";
    }
    if(!isNonNegativeFinite(p.b_accept_max_xy)){
        oss << "b_accept_max_xy invalid; ";
    }
    if(!isNonNegativeFinite(p.b_accept_max_yaw_rad)){
        oss << "b_accept_max_yaw_rad invalid; ";
    }
    if(!isNonNegativeFinite(p.b_accept_max_dq)){
        oss << "b_accept_max_dq invalid; ";
    }

    if(p.c_radii.empty()){
        oss << "c_radii empty; ";
    }else{
        for(std::size_t i = 0; i < p.c_radii.size(); ++i){
            if(!isPositiveFinite(p.c_radii[i])){
                oss << "c_radii[" << i << "] invalid; ";
            }
        }
    }
    if(!(p.c_yaw_bins > 0)){
        oss << "c_yaw_bins invalid; ";
    }
    if(p.c_yaw_offsets_rad.empty()){
        oss << "c_yaw_offsets_rad empty; ";
    }else{
        for(std::size_t i = 0; i < p.c_yaw_offsets_rad.size(); ++i){
            if(!std::isfinite(p.c_yaw_offsets_rad[i])){
                oss << "c_yaw_offsets_rad[" << i << "] invalid; ";
            }
        }
    }
    if(!(p.c_arm_max_iters > 0)){
        oss << "c_arm_max_iters invalid; ";
    }
    if(!isPositiveFinite(p.c_arm_max_ms)){
        oss << "c_arm_max_ms invalid; ";
    }
    if(!isPositiveFinite(p.c_max_ms)){
        oss << "c_max_ms invalid; ";
    }

    if(!isPositiveFinite(p.rank_w_xy)){
        oss << "rank_w_xy invalid; ";
    }
    if(!isPositiveFinite(p.rank_w_yaw)){
        oss << "rank_w_yaw invalid; ";
    }
    if(!isPositiveFinite(p.rank_w_q)){
        oss << "rank_w_q invalid; ";
    }
    if(!isPositiveFinite(p.rank_w_limit)){
        oss << "rank_w_limit invalid; ";
    }
    if(!isPositiveFinite(p.rank_w_gap)){
        oss << "rank_w_gap invalid; ";
    }
    if(!isPositiveFinite(p.rank_gap_ref)){
        oss << "rank_gap_ref invalid; ";
    }
    if(!isPositiveFinite(p.rank_joint_margin_req)){
        oss << "rank_joint_margin_req invalid; ";
    }
    if(!isPositiveFinite(p.task_rot_weight)){
        oss << "task_rot_weight invalid; ";
    }

    if(!isNonNegativeFinite(p.step_tol)){
        oss << "step_tol invalid; ";
    }
    if(!(p.stagnation_iters > 0)){
        oss << "stagnation_iters invalid; ";
    }
    if(!isPositiveFinite(p.lambda_init)){
        oss << "lambda_init invalid; ";
    }
    if(!isPositiveFinite(p.lambda_min)){
        oss << "lambda_min invalid; ";
    }
    if(!isPositiveFinite(p.lambda_max)){
        oss << "lambda_max invalid; ";
    }
    if(!(p.lambda_retry_max > 0)){
        oss << "lambda_retry_max invalid; ";
    }
    if(!(p.lambda_max >= p.lambda_init && p.lambda_max >= p.lambda_min)){
        oss << "lambda ordering invalid; ";
    }
    if(!(p.lambda_init >= p.lambda_min && p.lambda_init <= p.lambda_max)){
        oss << "lambda_init out of range; ";
    }

    reason = oss.str();
    return reason.empty();
}

// ################################
// C++: Stage B weighted DLS helpers begin
// ################################
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 9, 1> Vector9d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 9> Matrix69d;

void clampJoints9(Vector9d &xi,
                  const Eigen::VectorXd &q_min,
                  const Eigen::VectorXd &q_max)
{
    for(int i = 0; i < 6; ++i){
        xi(3 + i) = std::max(q_min(i), std::min(q_max(i), xi(3 + i)));
    }
}

void projectTrustAndJoints(Vector9d &xi,
                           const Vector9d &xi0,
                           const WholeBodyIkParams &p,
                           const Eigen::VectorXd &q_min,
                           const Eigen::VectorXd &q_max)
{
    xi(0) = std::max(xi0(0) - p.local_xy, std::min(xi0(0) + p.local_xy, xi(0)));
    xi(1) = std::max(xi0(1) - p.local_xy, std::min(xi0(1) + p.local_xy, xi(1)));
    xi(2) = wrapToPi(xi(2));
    double dyaw = wrapToPi(xi(2) - xi0(2));
    dyaw = std::max(-p.local_yaw_rad, std::min(p.local_yaw_rad, dyaw));
    xi(2) = wrapToPi(xi0(2) + dyaw);
    clampJoints9(xi, q_min, q_max);
}

bool jointsWithinLimits(const Vector9d &xi,
                        const Eigen::VectorXd &q_min,
                        const Eigen::VectorXd &q_max)
{
    for(int i = 0; i < 6; ++i){
        if(xi(3 + i) < q_min(i) || xi(3 + i) > q_max(i)){
            return false;
        }
    }
    return true;
}

double jointLimitMargin(const Vector9d &xi,
                        const Eigen::VectorXd &q_min,
                        const Eigen::VectorXd &q_max)
{
    double margin = std::numeric_limits<double>::infinity();
    for(int i = 0; i < 6; ++i){
        margin = std::min(margin, std::min(xi(3 + i) - q_min(i),
                                           q_max(i) - xi(3 + i)));
    }
    return std::isfinite(margin) ? margin : 0.0;
}

bool computeStageBError(const MMConfig::Ptr &cfg,
                        const Vector9d &xi,
                        const Eigen::Matrix4d &T_goal,
                        Vector6d &error,
                        double &pos_err,
                        double &rot_err)
{
    const Eigen::Vector3d car = xi.head<3>();
    const Eigen::VectorXd q = xi.tail<6>();
    const Eigen::Matrix4d T_now = cfg->getEePose(car, q);
    if(!T_now.allFinite()){
        return false;
    }
    poseError(T_now, T_goal, error);
    pos_err = error.head<3>().norm();
    rot_err = error.tail<3>().norm();
    return error.allFinite() && std::isfinite(pos_err) && std::isfinite(rot_err);
}

double weightedErrorSquared(const Vector6d &error, double rot_weight)
{
    Vector6d weighted = error;
    weighted.tail<3>() *= rot_weight;
    return weighted.squaredNorm();
}

void fillStageBCandidate(WholeBodyGoalCandidate &out_cand,
                         const Vector9d &xi,
                         const Vector9d &xi0,
                         double pos_err,
                         double rot_err,
                         const Eigen::VectorXd &q_min,
                         const Eigen::VectorXd &q_max)
{
    out_cand.base_xyyaw = xi.head<3>();
    out_cand.q = xi.tail<6>();
    out_cand.pos_err = pos_err;
    out_cand.rot_err_rad = rot_err;
    out_cand.base_xy_disp = (xi.head<2>() - xi0.head<2>()).norm();
    out_cand.yaw_disp = std::abs(wrapToPi(xi(2) - xi0(2)));
    out_cand.q_disp_norm = (xi.tail<6>() - xi0.tail<6>()).norm();
    out_cand.min_joint_margin = jointLimitMargin(xi, q_min, q_max);
    out_cand.obstacle_clearance = 0.0;
    out_cand.source = CandidateSource::B;
    out_cand.cost = std::numeric_limits<double>::infinity();
}

Eigen::Matrix<double, 9, 1> stageBWeightInverseSquared(const WholeBodyIkParams &p)
{
    Vector9d m_diag;
    const double m_xy = 1.0 / (p.b_weight_xy * p.b_weight_xy);
    const double m_yaw = 1.0 / (p.b_weight_yaw * p.b_weight_yaw);
    const double m_q = 1.0 / (p.b_weight_joint * p.b_weight_joint);
    m_diag << m_xy, m_xy, m_yaw, m_q, m_q, m_q, m_q, m_q, m_q;
    return m_diag;
}
// ################################
// C++: Stage B weighted DLS helpers end
// ################################

} // namespace

WholeBodyIkParams WholeBodyIkParams::loadFromRosParam(ros::NodeHandle &nh)
{
    WholeBodyIkParams p;

    nh.param("fsm/ee_goal_topic", p.ee_goal_topic, std::string("/ee_goal"));
    nh.param("fsm/ee_current_pose_topic",
             p.ee_current_pose_topic,
             std::string("/ee_current_pose"));

    nh.param("fsm/ee_ik_pos_tol", p.ik_pos_tol, 0.01);

    double ik_rot_tol_deg = 2.0;
    nh.param("fsm/ee_ik_rot_tol_deg", ik_rot_tol_deg, 2.0);
    p.ik_rot_tol_rad = ik_rot_tol_deg * kDeg2Rad;

    nh.param("fsm/ee_reach_pos_tol", p.reach_pos_tol, 0.02);

    double reach_rot_tol_deg = 4.0;
    nh.param("fsm/ee_reach_rot_tol_deg", reach_rot_tol_deg, 4.0);
    p.reach_rot_tol_rad = reach_rot_tol_deg * kDeg2Rad;

    nh.param("fsm/ee_ik_local_xy", p.local_xy, 0.40);

    double local_yaw_deg = 20.0;
    nh.param("fsm/ee_ik_local_yaw_deg", local_yaw_deg, 20.0);
    p.local_yaw_rad = local_yaw_deg * kDeg2Rad;

    nh.param("fsm/ee_ik_b_max_iters", p.b_max_iters, 60);
    nh.param("fsm/ee_ik_b_max_ms", p.b_max_ms, 40.0);

    nh.param("fsm/ee_ik_b_weight_xy", p.b_weight_xy, 5.0);
    nh.param("fsm/ee_ik_b_weight_yaw", p.b_weight_yaw, 2.0);
    nh.param("fsm/ee_ik_b_weight_joint", p.b_weight_joint, 1.0);

    nh.param("fsm/ee_ik_b_accept_joint_margin",
             p.b_accept_joint_margin,
             0.15);
    nh.param("fsm/ee_ik_b_accept_clearance", p.b_accept_clearance, 0.20);
    nh.param("fsm/ee_ik_b_accept_max_xy", p.b_accept_max_xy, 0.15);

    double b_accept_max_yaw_deg = 10.0;
    nh.param("fsm/ee_ik_b_accept_max_yaw_deg", b_accept_max_yaw_deg, 10.0);
    p.b_accept_max_yaw_rad = b_accept_max_yaw_deg * kDeg2Rad;

    nh.param("fsm/ee_ik_b_accept_max_dq", p.b_accept_max_dq, 0.50);

    nh.param("fsm/ee_ik_c_radii",
             p.c_radii,
             std::vector<double>{0.50, 0.75, 1.00, 1.25});
    nh.param("fsm/ee_ik_c_yaw_bins", p.c_yaw_bins, 12);
    p.c_yaw_offsets_rad = loadYawOffsetsRad(nh);

    nh.param("fsm/ee_ik_c_arm_iters", p.c_arm_max_iters, 40);
    nh.param("fsm/ee_ik_c_arm_max_ms", p.c_arm_max_ms, 5.0);
    nh.param("fsm/ee_ik_c_max_ms", p.c_max_ms, 150.0);

    nh.param("fsm/ee_ik_w_xy", p.rank_w_xy, 1.0);
    nh.param("fsm/ee_ik_w_yaw", p.rank_w_yaw, 0.3);
    nh.param("fsm/ee_ik_w_q", p.rank_w_q, 0.1);
    nh.param("fsm/ee_ik_w_limit", p.rank_w_limit, 1.0);
    nh.param("fsm/ee_ik_w_gap", p.rank_w_gap, 2.0);
    nh.param("fsm/ee_ik_gap_ref", p.rank_gap_ref, 0.20);
    nh.param("fsm/ee_ik_joint_margin_req", p.rank_joint_margin_req, 0.087);

    nh.param("fsm/ee_ik_task_rot_weight", p.task_rot_weight, 0.05);
    nh.param("fsm/ee_ik_step_tol", p.step_tol, 1e-4);
    nh.param("fsm/ee_ik_stagnation_iters", p.stagnation_iters, 5);
    nh.param("fsm/ee_ik_lambda_init", p.lambda_init, 1e-3);
    nh.param("fsm/ee_ik_lambda_min", p.lambda_min, 1e-6);
    nh.param("fsm/ee_ik_lambda_max", p.lambda_max, 1e6);
    nh.param("fsm/ee_ik_lambda_retry_max", p.lambda_retry_max, 5);

    std::string reason;
    if(!validateWholeBodyIkParams(p, reason)){
        ROS_FATAL("[EE IK] invalid WholeBodyIkParams: %s", reason.c_str());
        throw std::runtime_error("invalid WholeBodyIkParams");
    }

    return p;
}

WholeBodyIkSolver::WholeBodyIkSolver(MMConfig::Ptr cfg,
                                     GridMap::Ptr map,
                                     WholeBodyIkParams params)
    : cfg_(cfg),
      map_(map),
      params_(params),
      arm_ik_(cfg)
{
}

FixedBaseArmIkParams WholeBodyIkSolver::makeArmIkParams() const
{
    FixedBaseArmIkParams p;
    p.pos_tol = params_.ik_pos_tol;
    p.rot_tol_rad = params_.ik_rot_tol_rad;
    p.max_iters = params_.c_arm_max_iters;
    p.task_rot_weight = params_.task_rot_weight;
    p.joint_weight = params_.b_weight_joint;
    p.lambda_init = params_.lambda_init;
    p.lambda_min = params_.lambda_min;
    p.lambda_max = params_.lambda_max;
    p.lambda_retry_max = params_.lambda_retry_max;
    p.step_tol = params_.step_tol;
    p.stagnation_iters = params_.stagnation_iters;
    return p;
}

WholeBodyIkResult WholeBodyIkSolver::solve(const Eigen::Matrix<double, 9, 1> &xi_start,
                                           const Eigen::Matrix4d &T_goal)
{
    (void)xi_start;
    (void)T_goal;

    WholeBodyIkResult result;
    result.success = false;
    result.fail_reason = "not implemented";
    return result;
}

bool WholeBodyIkSolver::runStageB(const Eigen::Matrix<double, 9, 1> &xi0,
                                  const Eigen::Matrix4d &T_goal,
                                  WholeBodyGoalCandidate &out_cand)
{
    // ################################
    // C++: Stage B 9-DOF weighted DLS begin
    // ################################
    out_cand = WholeBodyGoalCandidate{};
    out_cand.source = CandidateSource::B;
    out_cand.q = Eigen::VectorXd::Zero(6);

    if(!cfg_ || xi0.size() != 9 || !xi0.allFinite() || !T_goal.allFinite()
       || cfg_->getManiDof() != 6
       || cfg_->getManipulatorMinPos().size() != 6
       || cfg_->getManipulatorMaxPos().size() != 6){
        out_cand.fail_reason = "invalid_input";
        return false;
    }

    const Eigen::VectorXd &q_min = cfg_->getManipulatorMinPos();
    const Eigen::VectorXd &q_max = cfg_->getManipulatorMaxPos();
    if(!q_min.allFinite() || !q_max.allFinite()
       || (q_min.array() > q_max.array()).any()){
        out_cand.fail_reason = "invalid_joint_limits";
        return false;
    }

    Vector9d xi = xi0;
    xi(2) = wrapToPi(xi(2));
    clampJoints9(xi, q_min, q_max);

    Vector6d error;
    double pos_err = std::numeric_limits<double>::infinity();
    double rot_err = std::numeric_limits<double>::infinity();
    if(!computeStageBError(cfg_, xi, T_goal, error, pos_err, rot_err)){
        fillStageBCandidate(out_cand, xi, xi0, pos_err, rot_err, q_min, q_max);
        out_cand.fail_reason = "non_finite_error";
        return false;
    }

    const Vector9d m_diag = stageBWeightInverseSquared(params_);
    double lambda = params_.lambda_init;
    int stagnation_count = 0;
    const ros::WallTime deadline =
        ros::WallTime::now() + ros::WallDuration(params_.b_max_ms * 1e-3);
    std::string fail_reason = "pose_not_reached";

    for(int iter = 0; iter < params_.b_max_iters; ++iter){
        if(pos_err <= params_.ik_pos_tol && rot_err <= params_.ik_rot_tol_rad){
            fail_reason.clear();
            break;
        }
        if(ros::WallTime::now() >= deadline){
            fail_reason = "timeout";
            break;
        }

        Matrix69d J;
        cfg_->getEeJacobian(xi.head<3>(), xi.tail<6>(), J);
        if(!J.allFinite()){
            fillStageBCandidate(out_cand, xi, xi0, pos_err, rot_err, q_min, q_max);
            out_cand.fail_reason = "non_finite_jacobian";
            return false;
        }

        Matrix6d S = Matrix6d::Identity();
        S.bottomRightCorner<3, 3>() *= params_.task_rot_weight;
        const Matrix69d A = S * J;
        const Vector6d b = S * error;
        if(!A.allFinite() || !b.allFinite()){
            fillStageBCandidate(out_cand, xi, xi0, pos_err, rot_err, q_min, q_max);
            out_cand.fail_reason = "non_finite";
            return false;
        }
        const double current_cost = b.squaredNorm();

        bool accepted = false;
        double accepted_step_norm = 0.0;
        for(int retry = 0; retry < params_.lambda_retry_max; ++retry){
            if(ros::WallTime::now() >= deadline){
                fail_reason = "timeout";
                break;
            }

            Matrix6d lhs = A * m_diag.asDiagonal() * A.transpose();
            lhs.diagonal().array() += lambda * lambda;
            const Eigen::LDLT<Matrix6d> ldlt(lhs);
            if(ldlt.info() != Eigen::Success){
                lambda = std::min(params_.lambda_max, lambda * 10.0);
                continue;
            }

            const Vector6d y = ldlt.solve(b);
            if(ldlt.info() != Eigen::Success || !y.allFinite()){
                lambda = std::min(params_.lambda_max, lambda * 10.0);
                continue;
            }

            const Vector9d delta = m_diag.asDiagonal() * A.transpose() * y;
            if(!delta.allFinite()){
                lambda = std::min(params_.lambda_max, lambda * 10.0);
                continue;
            }

            Vector9d xi_trial = xi + delta;
            projectTrustAndJoints(xi_trial, xi0, params_, q_min, q_max);
            if(map_ && !map_->isInMap(Eigen::Vector2d(xi_trial(0), xi_trial(1)))){
                lambda = std::min(params_.lambda_max, lambda * 10.0);
                continue;
            }

            Vector6d trial_error;
            double trial_pos = 0.0;
            double trial_rot = 0.0;
            if(!computeStageBError(cfg_, xi_trial, T_goal, trial_error,
                                   trial_pos, trial_rot)){
                lambda = std::min(params_.lambda_max, lambda * 10.0);
                continue;
            }

            if(weightedErrorSquared(trial_error, params_.task_rot_weight)
               < current_cost){
                accepted_step_norm = (xi_trial - xi).norm();
                xi = xi_trial;
                error = trial_error;
                pos_err = trial_pos;
                rot_err = trial_rot;
                lambda = std::max(params_.lambda_min, lambda / 3.0);
                accepted = true;
                break;
            }
            lambda = std::min(params_.lambda_max, lambda * 10.0);
        }

        if(fail_reason == "timeout"){
            break;
        }
        if(!accepted || accepted_step_norm <= params_.step_tol){
            ++stagnation_count;
        }else{
            stagnation_count = 0;
        }
        if(stagnation_count >= params_.stagnation_iters){
            fail_reason = "stagnation";
            break;
        }
    }

    fillStageBCandidate(out_cand, xi, xi0, pos_err, rot_err, q_min, q_max);

    const bool pose_ok = std::isfinite(pos_err) && std::isfinite(rot_err)
        && pos_err <= params_.ik_pos_tol
        && rot_err <= params_.ik_rot_tol_rad;
    const bool joints_ok = jointsWithinLimits(xi, q_min, q_max);
    const bool in_map = map_
        && map_->isInMap(Eigen::Vector2d(xi(0), xi(1)));
    const bool no_collision = cfg_
        && !cfg_->checkcollision(xi.head<3>(), xi.tail<6>(), true);

    out_cand.hard_valid = pose_ok && joints_ok && in_map && no_collision;
    if(pose_ok){
        if(!joints_ok){
            out_cand.fail_reason = "joint_limits";
        }else if(!in_map){
            out_cand.fail_reason = "out_of_map";
        }else if(!no_collision){
            out_cand.fail_reason = "collision";
        }else{
            out_cand.fail_reason.clear();
        }
        return true;
    }

    out_cand.fail_reason = fail_reason.empty() ? "pose_not_reached" : fail_reason;
    return false;
    // ################################
    // C++: Stage B 9-DOF weighted DLS end
    // ################################
}

bool WholeBodyIkSolver::runStageC(const Eigen::Matrix<double, 9, 1> &xi0,
                                  const Eigen::Matrix4d &T_goal,
                                  std::vector<WholeBodyGoalCandidate> &out_cands)
{
    (void)xi0;
    (void)T_goal;
    out_cands.clear();
    return false;
}

} // namespace remani_planner
// ################################
// C++: whole-body IK solver implementation end
// ################################
