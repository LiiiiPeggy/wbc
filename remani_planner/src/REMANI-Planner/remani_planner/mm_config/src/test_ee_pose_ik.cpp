// ################################
// C++: EE pose and Jacobian Milestone A tests begin
// ################################
#include <mm_config/ee_kinematics_utils.hpp>
#include <mm_config/mm_config.hpp>
#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <string>
#include <vector>

using remani_planner::MMConfig;
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

    std::cout << (all_pass ? "ALL TESTS PASSED" : "TESTS FAILED") << std::endl;
    return all_pass ? 0 : 1;
}
// ################################
// C++: EE pose and Jacobian Milestone A tests end
// ################################
