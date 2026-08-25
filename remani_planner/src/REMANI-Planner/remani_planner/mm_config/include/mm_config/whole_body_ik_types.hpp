// ################################
// C++: whole-body IK shared types begin
// ################################
#ifndef _WHOLE_BODY_IK_TYPES_HPP_
#define _WHOLE_BODY_IK_TYPES_HPP_

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <limits>
#include <string>
#include <vector>

namespace remani_planner
{

enum class CandidateSource { B, C };

struct WholeBodyGoalCandidate {
    Eigen::Vector3d base_xyyaw = Eigen::Vector3d::Zero();
    Eigen::VectorXd q;
    double pos_err = std::numeric_limits<double>::infinity();
    double rot_err_rad = std::numeric_limits<double>::infinity();
    double base_xy_disp = 0.0;
    double yaw_disp = 0.0;
    double q_disp_norm = 0.0;
    double min_joint_margin = 0.0;
    double obstacle_clearance = 0.0;
    bool hard_valid{false};
    double cost = std::numeric_limits<double>::infinity();
    CandidateSource source{CandidateSource::B};
    std::string fail_reason;
};

struct WholeBodyIkResult {
    bool success{false};
    Eigen::Matrix<double, 9, 1> xi_best = Eigen::Matrix<double, 9, 1>::Zero();
    WholeBodyGoalCandidate best;
    std::string fail_reason;
};

struct WholeBodyIkParams {
    // --- Topics (FSM uses directly; stored here for single source) ---
    std::string ee_goal_topic;
    std::string ee_current_pose_topic;

    // --- IK hard gates (solver terminal pose check) ---
    double ik_pos_tol;
    double ik_rot_tol_rad;

    // --- Actual FK reach (FSM may duplicate as ee_reach_* members; same YAML keys) ---
    double reach_pos_tol;
    double reach_rot_tol_rad;

    // --- Stage B trust region ---
    double local_xy;
    double local_yaw_rad;

    // --- Stage B iteration / time ---
    int b_max_iters;
    double b_max_ms;

    // --- Stage B configuration weights (W diagonal) ---
    double b_weight_xy;
    double b_weight_yaw;
    double b_weight_joint;

    // --- Stage B fast-path (skip Stage C when hard-valid B passes all) ---
    double b_accept_joint_margin;
    double b_accept_clearance;
    double b_accept_max_xy;
    double b_accept_max_yaw_rad;
    double b_accept_max_dq;

    // --- Stage C search geometry ---
    std::vector<double> c_radii;
    int c_yaw_bins;
    std::vector<double> c_yaw_offsets_rad;

    // --- Stage C arm IK per base candidate ---
    int c_arm_max_iters;
    double c_arm_max_ms;

    // --- Stage C total wall clock (ONLY cutoff — no early exit, no hidden cap) ---
    double c_max_ms;

    // --- Soft ranking weights ---
    double rank_w_xy;
    double rank_w_yaw;
    double rank_w_q;
    double rank_w_limit;
    double rank_w_gap;
    double rank_gap_ref;
    double rank_joint_margin_req;

    // --- Task-space rotation row weight (S matrix) ---
    double task_rot_weight;

    // --- LM / convergence numeric (must exist in struct — used by Stage B & FixedBaseArmIk) ---
    double step_tol;
    int stagnation_iters;
    double lambda_init;
    double lambda_min;
    double lambda_max;
    int lambda_retry_max;

    static WholeBodyIkParams loadFromRosParam(ros::NodeHandle &nh);
};

} // namespace remani_planner

#endif
// ################################
// C++: whole-body IK shared types end
// ################################
