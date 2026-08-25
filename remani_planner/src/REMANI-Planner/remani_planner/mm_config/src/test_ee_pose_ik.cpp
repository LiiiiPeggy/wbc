// ################################
// C++: EE pose and Jacobian Milestone A tests begin
// ################################
#include <mm_config/ee_kinematics_utils.hpp>
#include <mm_config/fixed_base_arm_ik.hpp>
#include <mm_config/mm_config.hpp>
#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <string>
#include <vector>

using remani_planner::MMConfig;
using remani_planner::FixedBaseArmIk;
using remani_planner::FixedBaseArmIkParams;
using remani_planner::FixedBaseArmIkResult;
using remani_planner::poseError;
using remani_planner::rotationLog;

namespace
{

const double kPi = 3.14159265358979323846;
const double kFdEps = 1e-7;

void ensureCr10TestParams(ros::NodeHandle &nh)
{
    if(nh.hasParam("mm/manipulator_type")){
        return;
    }

    nh.setParam("mm/manipulator_type", std::string("cr10"));
    nh.setParam("mm/mobile_base_dof", 2);
    nh.setParam("mm/mobile_base_length", 1.10);
    nh.setParam("mm/mobile_base_width", 0.90);
    nh.setParam("mm/mobile_base_height", 0.40);
    nh.setParam("mm/mobile_base_check_radius", 0.20);
    nh.setParam("mm/mobile_base_wheel_base", 0.56);
    nh.setParam("mm/mobile_base_wheel_radius", 0.125);
    nh.setParam("mm/mobile_base_max_wheel_omega", 4.0);
    nh.setParam("mm/mobile_base_max_wheel_alpha", 8.0);
    nh.setParam("mm/manipulator_dof", 6);
    nh.setParam("mm/manipulator_thickness", 0.06);
    nh.setParam("mm/manipulator_config",
                std::vector<double>{0.1765, 0.607, 0.568, 0.191, 0.125, 0.1084});
    nh.setParam("mm/manipulator_min_pos",
                std::vector<double>{-3, -1.5, -2.8, -3, -3, -3});
    nh.setParam("mm/manipulator_max_pos",
                std::vector<double>{3, 1.5, 2.8, 3, 3, 3});
    nh.setParam("mm/base_mani_fixed_joint_xyz_ypr",
                std::vector<double>{0.2462, 0.0, 0.1, 0.0, 0.0, 0.0});
    nh.setParam("optimization/safe_margin", 0.05);
    nh.setParam("optimization/safe_margin_mani", 0.05);
    nh.setParam("optimization/self_safe_margin", 0.02);
    nh.setParam("optimization/ground_safe_dis", 0.1);
    nh.setParam("mm/ee_tcp_xyz_rpy", std::vector<double>{0, 0, 0, 0, 0, 0});
}

Eigen::Matrix3d urdfRpy(double roll, double pitch, double yaw)
{
    const Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                               * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                               * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    return q.toRotationMatrix();
}

Eigen::Matrix4d makeTransform(double x, double y, double z,
                              double roll, double pitch, double yaw)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = urdfRpy(roll, pitch, yaw);
    T.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);
    return T;
}

Eigen::Matrix4d rotateZ(double angle)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) =
        Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return T;
}

Eigen::Matrix4d carTransform(const Eigen::Vector3d &car)
{
    return makeTransform(car.x(), car.y(), 0.0, 0.0, 0.0, car.z());
}

Eigen::Matrix4d cr10ReferencePose(const Eigen::Vector3d &car,
                                  const Eigen::VectorXd &q,
                                  const Eigen::Matrix4d &T_tcp)
{
    const Eigen::Matrix4d fixed[6] = {
        makeTransform(0, 0, 0.1765, 0, 0, 0),
        makeTransform(0, 0, 0, 1.5708, 1.5708, 0),
        makeTransform(-0.607, 0, 0, 0, 0, 0),
        makeTransform(-0.568, 0, 0.191, 0, 0, -1.5708),
        makeTransform(0, -0.125, 0, 1.5708, 0, 0),
        makeTransform(0, 0.1084, 0, -1.5708, 0, 0)
    };

    Eigen::Matrix4d T = carTransform(car)
                      * makeTransform(0.2462, 0, 0.1, 0, 0, 0);
    for(int i = 0; i < 6; ++i){
        T = T * fixed[i] * rotateZ(q(i));
    }
    return T * T_tcp;
}

Eigen::Matrix<double, 6, 9> finiteDifferenceJacobian(
    MMConfig &cfg, const Eigen::Vector3d &car, const Eigen::VectorXd &q)
{
    Eigen::Matrix<double, 6, 9> J_fd;
    const Eigen::Matrix4d T0 = cfg.getEePose(car, q);
    const Eigen::Vector3d p0 = T0.block<3, 1>(0, 3);
    const Eigen::Matrix3d R0 = T0.block<3, 3>(0, 0);

    for(int i = 0; i < 9; ++i){
        Eigen::Vector3d car_plus = car;
        Eigen::VectorXd q_plus = q;
        if(i < 3){
            car_plus(i) += kFdEps;
        }else{
            q_plus(i - 3) += kFdEps;
        }

        const Eigen::Matrix4d T_plus = cfg.getEePose(car_plus, q_plus);
        J_fd.block<3, 1>(0, i) =
            (T_plus.block<3, 1>(0, 3) - p0) / kFdEps;
        J_fd.block<3, 1>(3, i) =
            rotationLog(T_plus.block<3, 3>(0, 0) * R0.transpose()) / kFdEps;
    }
    return J_fd;
}

bool jacobianColumnsPass(MMConfig &cfg, const Eigen::Vector3d &car,
                         const Eigen::VectorXd &q, double &max_abs,
                         double &max_rel)
{
    Eigen::Matrix<double, 6, 9> J;
    cfg.getEeJacobian(car, q, J);
    const Eigen::Matrix<double, 6, 9> J_fd =
        finiteDifferenceJacobian(cfg, car, q);

    bool pass = J.allFinite() && J_fd.allFinite();
    for(int i = 0; i < 9; ++i){
        const double abs_err = (J.col(i) - J_fd.col(i)).norm();
        const double rel_err = abs_err / std::max(1.0, J_fd.col(i).norm());
        max_abs = std::max(max_abs, abs_err);
        max_rel = std::max(max_rel, rel_err);
        if(!(abs_err < 1e-4 || rel_err < 1e-4)){
            ROS_ERROR("Jacobian column %d failed: abs=%.6e rel=%.6e",
                      i, abs_err, rel_err);
            pass = false;
        }
    }
    return pass;
}

void printResult(const std::string &name, bool pass, bool &all_pass)
{
    std::cout << (pass ? "[PASS] " : "[FAIL] ") << name << std::endl;
    all_pass = all_pass && pass;
}

} // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_ee_pose_ik");
    ros::NodeHandle nh;
    ensureCr10TestParams(nh);
    nh.setParam("mm/ee_tcp_xyz_rpy", std::vector<double>{0, 0, 0, 0, 0, 0});

    MMConfig cfg_identity;
    cfg_identity.setParam(nh);
    bool all_pass = true;

    const Eigen::Matrix4d identity_tcp = cfg_identity.getTcpTransform();
    printResult("TCP identity regression",
                (identity_tcp - Eigen::Matrix4d::Identity()).norm() < 1e-12,
                all_pass);

    std::mt19937 generator(20260819);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    std::uniform_real_distribution<double> base_xy(-2.0, 2.0);
    std::uniform_real_distribution<double> base_yaw(-kPi, kPi);
    const Eigen::VectorXd q_min = cfg_identity.getManipulatorMinPos();
    const Eigen::VectorXd q_max = cfg_identity.getManipulatorMaxPos();

    double max_fk_pos = 0.0;
    double max_fk_rot_deg = 0.0;
    bool fk_pass = true;
    for(int sample = 0; sample < 100; ++sample){
        const Eigen::Vector3d car(base_xy(generator), base_xy(generator),
                                  base_yaw(generator));
        Eigen::VectorXd q(6);
        for(int i = 0; i < 6; ++i){
            q(i) = q_min(i) + unit(generator) * (q_max(i) - q_min(i));
        }

        const Eigen::Matrix4d T_actual = cfg_identity.getEePose(car, q);
        const Eigen::Matrix4d T_reference =
            cr10ReferencePose(car, q, Eigen::Matrix4d::Identity());
        Eigen::Matrix<double, 6, 1> error;
        poseError(T_actual, T_reference, error);
        const double pos_err = error.head<3>().norm();
        const double rot_err_deg = error.tail<3>().norm() * 180.0 / kPi;
        max_fk_pos = std::max(max_fk_pos, pos_err);
        max_fk_rot_deg = std::max(max_fk_rot_deg, rot_err_deg);
        fk_pass = fk_pass && pos_err < 1e-3 && rot_err_deg < 0.1;
    }
    ROS_INFO("random FK: max position=%.6e m, max rotation=%.6e deg",
             max_fk_pos, max_fk_rot_deg);
    printResult("100-sample random whole-body FK", fk_pass, all_pass);

    Eigen::Vector3d car(0.7, -0.4, 0.35);
    Eigen::VectorXd q(6);
    q << 0.4, -0.8, 1.1, -0.6, 0.75, -0.3;

    Eigen::VectorXd q5_changed = q;
    q5_changed(5) += 0.2;
    Eigen::Matrix<double, 6, 1> q5_pose_error;
    poseError(cfg_identity.getEePose(car, q),
              cfg_identity.getEePose(car, q5_changed), q5_pose_error);
    printResult("q5 changes EE orientation",
                q5_pose_error.tail<3>().norm() > 0.1, all_pass);

    double max_jac_abs = 0.0;
    double max_jac_rel = 0.0;
    bool jacobian_pass = true;
    for(int sample = 0; sample < 20; ++sample){
        Eigen::Vector3d random_car(base_xy(generator), base_xy(generator),
                                   base_yaw(generator));
        Eigen::VectorXd random_q(6);
        for(int i = 0; i < 6; ++i){
            const double margin = 0.05 * (q_max(i) - q_min(i));
            random_q(i) = q_min(i) + margin
                        + unit(generator) * (q_max(i) - q_min(i) - 2.0 * margin);
        }
        jacobian_pass = jacobianColumnsPass(
            cfg_identity, random_car, random_q, max_jac_abs, max_jac_rel)
            && jacobian_pass;
    }
    ROS_INFO("identity TCP Jacobian: max abs=%.6e max rel=%.6e",
             max_jac_abs, max_jac_rel);
    printResult("6x9 finite-difference Jacobian", jacobian_pass, all_pass);

    Eigen::Matrix<double, 6, 9> J_identity;
    cfg_identity.getEeJacobian(car, q, J_identity);
    printResult("q5 Jacobian column is non-zero",
                J_identity.col(8).norm() > 0.5, all_pass);

    const std::vector<double> tcp_values{0.12, -0.04, 0.08, 0.10, -0.20, 0.15};
    nh.setParam("mm/ee_tcp_xyz_rpy", tcp_values);
    MMConfig cfg_offset;
    cfg_offset.setParam(nh);

    const Eigen::Matrix4d T_identity = cfg_identity.getEePose(car, q);
    const Eigen::Matrix4d T_offset = cfg_offset.getEePose(car, q);
    Eigen::Matrix<double, 6, 9> J_offset;
    cfg_offset.getEeJacobian(car, q, J_offset);
    double offset_max_abs = 0.0;
    double offset_max_rel = 0.0;
    const bool offset_fd_pass = jacobianColumnsPass(
        cfg_offset, car, q, offset_max_abs, offset_max_rel);
    const bool tcp_smoke_pass =
        (T_offset.block<3, 1>(0, 3) - T_identity.block<3, 1>(0, 3)).norm() > 1e-3
        && (J_offset.topRows<3>() - J_identity.topRows<3>()).norm() > 1e-3
        && (J_offset.bottomRows<3>() - J_identity.bottomRows<3>()).norm() < 1e-9
        && offset_fd_pass;
    ROS_INFO("offset TCP Jacobian: max abs=%.6e max rel=%.6e",
             offset_max_abs, offset_max_rel);
    printResult("non-zero TCP pose/linear change and angular FD",
                tcp_smoke_pass, all_pass);

    // ################################
    // C++: fixed-base 6-DoF arm IK tests begin
    // ################################
    nh.setParam("mm/ee_tcp_xyz_rpy", std::vector<double>{0, 0, 0, 0, 0, 0});
    MMConfig::Ptr cfg_ik(new MMConfig());
    cfg_ik->setParam(nh);
    FixedBaseArmIk arm_ik(cfg_ik);

    FixedBaseArmIkParams arm_p;
    arm_p.pos_tol = 1e-4;
    arm_p.rot_tol_rad = 1e-3;
    arm_p.max_iters = 100;
    arm_p.task_rot_weight = 0.5;
    arm_p.joint_weight = 1.0;
    arm_p.lambda_init = 1e-3;
    arm_p.lambda_min = 1e-6;
    arm_p.lambda_max = 1e3;
    arm_p.lambda_retry_max = 8;
    arm_p.step_tol = 1e-9;
    arm_p.stagnation_iters = 8;

    const Eigen::Vector3d ik_car(0.25, -0.15, 0.2);
    Eigen::VectorXd q_goal(6);
    q_goal << 0.35, -0.65, 0.95, -0.45, 0.55, -0.25;
    Eigen::VectorXd q_seed = q_goal;
    q_seed << q_goal(0) + 0.08, q_goal(1) - 0.06,
              q_goal(2) + 0.05, q_goal(3) - 0.07,
              q_goal(4) + 0.04, q_goal(5) - 0.05;
    for(int i = 0; i < 6; ++i){
        q_seed(i) = std::max(q_min(i), std::min(q_max(i), q_seed(i)));
    }

    const Eigen::Matrix4d T_goal = cfg_ik->getEePose(ik_car, q_goal);
    const FixedBaseArmIkResult ik_result = arm_ik.solve(
        ik_car, T_goal, q_seed, arm_p,
        ros::WallTime::now() + ros::WallDuration(1.0));
    Eigen::Matrix<double, 6, 1> ik_pose_error;
    poseError(cfg_ik->getEePose(ik_car, ik_result.q), T_goal, ik_pose_error);
    bool ik_limits_pass = ik_result.q.size() == 6 && ik_result.q.allFinite();
    for(int i = 0; i < ik_result.q.size() && ik_limits_pass; ++i){
        ik_limits_pass = ik_result.q(i) >= q_min(i)
                      && ik_result.q(i) <= q_max(i);
    }
    const bool ik_success_pass = ik_result.success
        && ik_limits_pass
        && std::isfinite(ik_result.pos_err)
        && std::isfinite(ik_result.rot_err_rad)
        && ik_pose_error.head<3>().norm() <= arm_p.pos_tol
        && ik_pose_error.tail<3>().norm() <= arm_p.rot_tol_rad;
    printResult("fixed-base arm IK reaches forward-FK goal",
                ik_success_pass, all_pass);

    Eigen::Matrix4d T_unreachable = Eigen::Matrix4d::Identity();
    T_unreachable.block<3, 1>(0, 3) = Eigen::Vector3d(100.0, 100.0, 100.0);
    const ros::WallTime failure_start = ros::WallTime::now();
    const ros::WallTime failure_deadline =
        failure_start + ros::WallDuration(0.25);
    const FixedBaseArmIkResult failure_result = arm_ik.solve(
        ik_car, T_unreachable, q_seed, arm_p, failure_deadline);
    bool failure_limits_pass =
        failure_result.q.size() == 6 && failure_result.q.allFinite();
    for(int i = 0; i < failure_result.q.size() && failure_limits_pass; ++i){
        failure_limits_pass = failure_result.q(i) >= q_min(i)
                           && failure_result.q(i) <= q_max(i);
    }
    const bool ik_failure_pass = !failure_result.success
        && !failure_result.fail_reason.empty()
        && failure_limits_pass
        && std::isfinite(failure_result.pos_err)
        && std::isfinite(failure_result.rot_err_rad)
        && ros::WallTime::now() <= failure_deadline + ros::WallDuration(0.1);
    printResult("fixed-base arm IK rejects unreachable position",
                ik_failure_pass, all_pass);
    // ################################
    // C++: fixed-base 6-DoF arm IK tests end
    // ################################

    std::cout << (all_pass ? "ALL TESTS PASSED" : "TESTS FAILED") << std::endl;
    return all_pass ? 0 : 1;
}
// ################################
// C++: EE pose and Jacobian Milestone A tests end
// ################################
