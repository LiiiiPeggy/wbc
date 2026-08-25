// ################################
// C++: fixed-base 6-DoF arm IK API begin
// ################################
#ifndef _FIXED_BASE_ARM_IK_HPP_
#define _FIXED_BASE_ARM_IK_HPP_

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <limits>
#include <string>
#include <mm_config/mm_config.hpp>

namespace remani_planner
{

struct FixedBaseArmIkParams {
    double pos_tol;
    double rot_tol_rad;
    int max_iters;

    double task_rot_weight;
    double joint_weight;

    double lambda_init;
    double lambda_min;
    double lambda_max;
    int lambda_retry_max;

    double step_tol;
    int stagnation_iters;
};

struct FixedBaseArmIkResult {
    bool success{false};

    Eigen::VectorXd q;

    double pos_err{std::numeric_limits<double>::infinity()};
    double rot_err_rad{std::numeric_limits<double>::infinity()};

    int iters{0};

    std::string fail_reason;
};

class FixedBaseArmIk {
public:
    explicit FixedBaseArmIk(const MMConfig::Ptr &cfg);

    FixedBaseArmIkResult solve(
        const Eigen::Vector3d &car_xyyaw,
        const Eigen::Matrix4d &T_goal,
        const Eigen::VectorXd &q_seed,
        const FixedBaseArmIkParams &p,
        const ros::WallTime &deadline);

private:
    MMConfig::Ptr cfg_;
};

} // namespace remani_planner

#endif
// ################################
// C++: fixed-base 6-DoF arm IK API end
// ################################
