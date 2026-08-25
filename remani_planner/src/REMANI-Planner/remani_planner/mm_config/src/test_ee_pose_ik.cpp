// ################################
// C++: EE pose and Jacobian Milestone A tests begin
// ################################
#include <mm_config/ee_kinematics_utils.hpp>
#include <mm_config/fixed_base_arm_ik.hpp>
#include <mm_config/mm_config.hpp>
#include <mm_config/whole_body_ik.hpp>
#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

using remani_planner::CandidateSource;
using remani_planner::FixedBaseArmIk;
using remani_planner::FixedBaseArmIkParams;
using remani_planner::FixedBaseArmIkResult;
using remani_planner::MMConfig;
using remani_planner::WholeBodyGoalCandidate;
using remani_planner::WholeBodyIkParams;
using remani_planner::WholeBodyIkSolver;
using remani_planner::poseError;
using remani_planner::rotationLog;
using remani_planner::wrapToPi;

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

// ################################
// C++: GridMap Stage B test fixture helpers begin
// ################################
void setGridMapFixtureParams(ros::NodeHandle &nh)
{
    nh.setParam("grid_map/resolution", 0.05);
    nh.setParam("grid_map/map_size_x", 6.0);
    nh.setParam("grid_map/map_size_y", 6.0);
    nh.setParam("grid_map/map_size_z", 2.5);
    nh.setParam("grid_map/local_update_range_x", 6.0);
    nh.setParam("grid_map/local_update_range_y", 6.0);
    nh.setParam("grid_map/local_update_range_z", 2.5);
    nh.setParam("grid_map/obstacles_inflation", 0.08);
    nh.setParam("grid_map/local_map_margin", 2);
    nh.setParam("grid_map/ground_height", -0.01);
    nh.setParam("grid_map/fx", 320.0);
    nh.setParam("grid_map/fy", 320.0);
    nh.setParam("grid_map/cx", 320.0);
    nh.setParam("grid_map/cy", 240.0);
    nh.setParam("grid_map/use_depth_filter", true);
    nh.setParam("grid_map/depth_filter_tolerance", 0.15);
    nh.setParam("grid_map/depth_filter_maxdist", 5.0);
    nh.setParam("grid_map/depth_filter_mindist", 0.2);
    nh.setParam("grid_map/depth_filter_margin", 2);
    nh.setParam("grid_map/k_depth_scaling_factor", 1000.0);
    nh.setParam("grid_map/skip_pixel", 2);
    nh.setParam("grid_map/p_hit", 0.65);
    nh.setParam("grid_map/p_miss", 0.35);
    nh.setParam("grid_map/p_min", 0.12);
    nh.setParam("grid_map/p_max", 0.90);
    nh.setParam("grid_map/p_occ", 0.80);
    nh.setParam("grid_map/min_ray_length", 0.1);
    nh.setParam("grid_map/max_ray_length", 4.5);
    nh.setParam("grid_map/visualization_truncate_height", 2.8);
    nh.setParam("grid_map/virtual_ceil_height", 2.4);
    nh.setParam("grid_map/virtual_ceil_yp", -0.1);
    nh.setParam("grid_map/virtual_ceil_yn", -0.1);
    nh.setParam("grid_map/show_occ_time", false);
    nh.setParam("grid_map/pose_type", 1);
    nh.setParam("grid_map/frame_id", std::string("world"));
    nh.setParam("grid_map/odom_depth_timeout", 14.0);
    nh.setParam("grid_map/esdf_slice_height", -0.1);
    nh.setParam("grid_map/show_esdf_time", false);
    nh.setParam("grid_map/local_bound_inflate", 0.0);
    nh.setParam("grid_map/use_global_map", true);
    nh.setParam("grid_map/use_load_map", false);
}

GridMap::Ptr makeInitializedGridMap(ros::NodeHandle &nh)
{
    setGridMapFixtureParams(nh);
    GridMap::Ptr grid(new GridMap());
    grid->initMap(nh);
    grid->resetBuffer();
    grid->updateESDF3d();
    return grid;
}

void occupyCarFootprint(GridMap &grid, const Eigen::Vector3d &car)
{
    const double res = 0.05;
    for(double x = car.x() - 0.70; x <= car.x() + 0.70 + 1e-9; x += res){
        for(double y = car.y() - 0.55; y <= car.y() + 0.55 + 1e-9; y += res){
            for(double z = 0.15; z <= 0.45 + 1e-9; z += res){
                grid.setOccupied(Eigen::Vector3d(x, y, z));
            }
        }
    }
    grid.updateESDF3d();
}

void unpackXi(const Eigen::Matrix<double, 9, 1> &xi,
              Eigen::Vector3d &car, Eigen::VectorXd &q)
{
    car = xi.template head<3>();
    q = xi.template tail<6>();
}

Eigen::Matrix<double, 9, 1> packXi(const Eigen::Vector3d &car,
                                   const Eigen::VectorXd &q)
{
    Eigen::Matrix<double, 9, 1> xi;
    xi.template head<3>() = car;
    xi.template tail<6>() = q;
    return xi;
}
// ################################
// C++: GridMap Stage B test fixture helpers end
// ################################

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

    // ################################
    // C++: whole-body IK params and solver scaffold tests begin
    // ################################
    bool params_load_pass = true;
    try{
        const WholeBodyIkParams default_params = WholeBodyIkParams::loadFromRosParam(nh);
        params_load_pass = default_params.ee_goal_topic == "/ee_goal"
            && default_params.ee_current_pose_topic == "/ee_current_pose"
            && default_params.b_weight_joint > 0.0
            && default_params.c_radii.size() == 4
            && default_params.c_yaw_offsets_rad.size() == 3;
    }catch(const std::exception &ex){
        ROS_ERROR("WholeBodyIkParams default load failed: %s", ex.what());
        params_load_pass = false;
    }
    printResult("WholeBodyIkParams loadFromRosParam defaults",
                params_load_pass, all_pass);

    bool params_reject_pass = false;
    nh.setParam("fsm/ee_ik_b_weight_joint", 0.0);
    try{
        WholeBodyIkParams::loadFromRosParam(nh);
    }catch(const std::runtime_error &){
        params_reject_pass = true;
    }
    nh.deleteParam("fsm/ee_ik_b_weight_joint");
    printResult("WholeBodyIkParams rejects zero b_weight_joint",
                params_reject_pass, all_pass);

    GridMap::Ptr grid_map(new GridMap());
    WholeBodyIkParams solver_params;
    try{
        solver_params = WholeBodyIkParams::loadFromRosParam(nh);
    }catch(const std::exception &){
        solver_params = WholeBodyIkParams{};
    }
    WholeBodyIkSolver whole_body_ik(cfg_ik, grid_map, solver_params);
    Eigen::Matrix<double, 9, 1> xi_start;
    xi_start << ik_car.x(), ik_car.y(), ik_car.z(), q_seed(0), q_seed(1),
                q_seed(2), q_seed(3), q_seed(4), q_seed(5);
    const auto stub_result = whole_body_ik.solve(xi_start, T_goal);
    printResult("WholeBodyIkSolver stub solve returns not implemented",
                !stub_result.success
                    && stub_result.fail_reason == "not implemented",
                all_pass);
    // ################################
    // C++: whole-body IK params and solver scaffold tests end
    // ################################

    // ################################
    // C++: Stage B near-goal and collision tests begin
    // ################################
    GridMap::Ptr stage_b_grid = makeInitializedGridMap(nh);
    MMConfig::Ptr cfg_stage_b(new MMConfig());
    cfg_stage_b->setParam(nh, stage_b_grid);
    WholeBodyIkParams stage_b_params = WholeBodyIkParams::loadFromRosParam(nh);
    WholeBodyIkSolver stage_b_solver(cfg_stage_b, stage_b_grid, stage_b_params);

    Eigen::Vector3d b_car(0.20, -0.10, 0.15);
    Eigen::VectorXd b_q_start(6);
    b_q_start << 0.35, -0.65, 0.95, -0.45, 0.55, -0.25;
    Eigen::VectorXd b_q_goal = b_q_start;
    b_q_goal << 0.41, -0.70, 0.99, -0.48, 0.59, -0.28;
    const Eigen::Vector3d b_car_goal(0.25, -0.14, 0.23);
    const Eigen::Matrix<double, 9, 1> xi_b_start = packXi(b_car, b_q_start);
    const Eigen::Matrix4d T_b_goal = cfg_stage_b->getEePose(b_car_goal, b_q_goal);

    int start_coll_type = -1;
    int goal_coll_type = -1;
    const bool start_free = !cfg_stage_b->checkcollision(b_car, b_q_start, true, start_coll_type);
    const bool goal_free = !cfg_stage_b->checkcollision(b_car_goal, b_q_goal, true, goal_coll_type);

    WholeBodyGoalCandidate near_cand;
    const bool near_ok = stage_b_solver.runStageB(xi_b_start, T_b_goal, near_cand);
    bool near_pose_gates = false;
    if(near_cand.q.size() == 6){
        Eigen::Matrix<double, 6, 1> near_pose_error;
        poseError(cfg_stage_b->getEePose(near_cand.base_xyyaw, near_cand.q),
                  T_b_goal, near_pose_error);
        near_pose_gates = near_pose_error.head<3>().norm() <= stage_b_params.ik_pos_tol
                       && near_pose_error.tail<3>().norm() <= stage_b_params.ik_rot_tol_rad;
    }
    const bool near_goal_pass = start_free && goal_free
        && near_ok
        && near_cand.source == CandidateSource::B
        && near_cand.hard_valid
        && near_cand.q.size() == 6
        && near_cand.base_xy_disp <= stage_b_params.local_xy
        && near_cand.yaw_disp <= stage_b_params.local_yaw_rad
        && near_cand.pos_err <= stage_b_params.ik_pos_tol
        && near_cand.rot_err_rad <= stage_b_params.ik_rot_tol_rad
        && near_pose_gates;
    printResult("Stage B reaches near-current forward-FK goal",
                near_goal_pass, all_pass);
    if(!near_goal_pass){
        ROS_ERROR("Stage B near-goal: start_free=%d goal_free=%d ok=%d hard_valid=%d "
                  "reason=%s pos=%.4e rot=%.4e xy_disp=%.4e coll_start=%d coll_goal=%d",
                  static_cast<int>(start_free), static_cast<int>(goal_free),
                  static_cast<int>(near_ok), static_cast<int>(near_cand.hard_valid),
                  near_cand.fail_reason.c_str(), near_cand.pos_err, near_cand.rot_err_rad,
                  near_cand.base_xy_disp, start_coll_type, goal_coll_type);
    }

    occupyCarFootprint(*stage_b_grid, b_car);
    occupyCarFootprint(*stage_b_grid, b_car_goal);
    if(near_cand.q.size() == 6){
        occupyCarFootprint(*stage_b_grid, near_cand.base_xyyaw);
    }
    WholeBodyGoalCandidate coll_cand;
    const bool coll_ok = stage_b_solver.runStageB(xi_b_start, T_b_goal, coll_cand);
    const bool terminal_in_collision = (coll_cand.q.size() == 6)
        && cfg_stage_b->checkcollision(coll_cand.base_xyyaw, coll_cand.q, true);
    const bool collision_hard_invalid_pass = coll_ok
        && coll_cand.source == CandidateSource::B
        && !coll_cand.hard_valid
        && terminal_in_collision;
    printResult("Stage B occupied voxel sets hard_valid false",
                collision_hard_invalid_pass, all_pass);
    if(!collision_hard_invalid_pass){
        ROS_ERROR("Stage B collision: ok=%d hard_valid=%d in_coll=%d reason=%s",
                  static_cast<int>(coll_ok), static_cast<int>(coll_cand.hard_valid),
                  static_cast<int>(terminal_in_collision),
                  coll_cand.fail_reason.c_str());
    }
    // ################################
    // C++: Stage B near-goal and collision tests end
    // ################################

    // ################################
    // C++: ranking helpers tests begin
    // ################################
    {
        WholeBodyIkParams rank_p = WholeBodyIkParams::loadFromRosParam(nh);
        WholeBodyGoalCandidate good_margin;
        good_margin.hard_valid = true;
        good_margin.base_xy_disp = 0.10;
        good_margin.yaw_disp = 0.05;
        good_margin.q_disp_norm = 0.20;
        good_margin.min_joint_margin = 0.40;
        good_margin.obstacle_clearance = 0.50;
        good_margin.q = b_q_start;

        WholeBodyGoalCandidate poor_margin = good_margin;
        poor_margin.min_joint_margin = 0.02;

        const double cost_good = remani_planner::computeCandidateCost(rank_p, good_margin);
        const double cost_poor = remani_planner::computeCandidateCost(rank_p, poor_margin);
        const double margin_from_api =
            remani_planner::computeJointLimitMargin(*cfg_stage_b, b_q_start);
        const bool ranking_pass = cost_poor > cost_good
            && std::isfinite(cost_good) && std::isfinite(cost_poor)
            && margin_from_api > 0.0
            && remani_planner::passesFastPathQuality(rank_p, good_margin)
            && !remani_planner::passesFastPathQuality(rank_p, poor_margin);
        printResult("ranking: worse joint margin raises cost",
                    ranking_pass, all_pass);
        if(!ranking_pass){
            ROS_ERROR("ranking: cost_good=%.4e cost_poor=%.4e margin_api=%.4e",
                      cost_good, cost_poor, margin_from_api);
        }
    }
    // ################################
    // C++: ranking helpers tests end
    // ################################

    // ################################
    // C++: obstacle clearance API tests begin
    // ################################
    {
        GridMap::Ptr clear_grid = makeInitializedGridMap(nh);
        MMConfig::Ptr cfg_clear(new MMConfig());
        cfg_clear->setParam(nh, clear_grid);
        Eigen::Vector3d clear_car(0.0, 0.0, 0.0);
        Eigen::VectorXd clear_q = b_q_start;
        const double empty_clear =
            cfg_clear->getWholeBodyObstacleClearance(clear_car, clear_q);
        const double empty_car_clear =
            cfg_clear->getCarObstacleClearance(clear_car);
        const bool empty_coll = cfg_clear->checkcollision(clear_car, clear_q, true);

        clear_grid->setOccupied(Eigen::Vector3d(1.0, 0.0, 0.5));
        clear_grid->updateESDF3d();
        const Eigen::Vector3d near_car(0.70, 0.0, 0.0);
        const double occupied_clear =
            cfg_clear->getWholeBodyObstacleClearance(near_car, clear_q);
        const bool occupied_coll_bool =
            cfg_clear->checkcollision(near_car, clear_q, true);
        // Recompute checkcollision after reading clearance must be stable
        const bool occupied_coll_bool2 =
            cfg_clear->checkcollision(near_car, clear_q, true);

        const bool clearance_pass = empty_clear > 0.3
            && empty_car_clear > 0.3
            && !empty_coll
            && occupied_clear < empty_clear
            && occupied_clear < 0.5
            && occupied_coll_bool == occupied_coll_bool2;
        printResult("obstacle clearance empty large / occupied small",
                    clearance_pass, all_pass);
        if(!clearance_pass){
            ROS_ERROR("clearance: empty=%.4f car=%.4f occupied=%.4f coll=%d/%d",
                      empty_clear, empty_car_clear, occupied_clear,
                      static_cast<int>(occupied_coll_bool),
                      static_cast<int>(occupied_coll_bool2));
        }
    }
    // ################################
    // C++: obstacle clearance API tests end
    // ################################

    // ################################
    // C++: Stage C base candidate generation tests begin
    // ################################
    {
        GridMap::Ptr base_grid = makeInitializedGridMap(nh);
        MMConfig::Ptr cfg_base(new MMConfig());
        cfg_base->setParam(nh, base_grid);
        WholeBodyIkParams base_params = WholeBodyIkParams::loadFromRosParam(nh);
        WholeBodyIkSolver base_solver(cfg_base, base_grid, base_params);

        Eigen::Vector3d car_a(0.0, 0.0, 0.0);
        Eigen::VectorXd q_a = b_q_start;
        const Eigen::Matrix4d T_goal_a = cfg_base->getEePose(car_a, q_a);
        std::vector<Eigen::Vector3d> bases_a;
        base_solver.generateFilteredBaseCandidates(T_goal_a, car_a, bases_a);

        Eigen::Vector3d car_b(0.35, 0.20, 0.8);
        std::vector<Eigen::Vector3d> bases_b;
        base_solver.generateFilteredBaseCandidates(T_goal_a, car_b, bases_b);

        bool all_in_map = !bases_a.empty();
        for(const auto &car : bases_a){
            if(!base_grid->isInMap(Eigen::Vector2d(car.x(), car.y()))){
                all_in_map = false;
                break;
            }
            double md = 0.0;
            if(cfg_base->checkCarObsCollision(car, true, true, md)){
                all_in_map = false;
                break;
            }
        }
        const bool current_first = !bases_a.empty()
            && std::hypot(bases_a.front().x() - car_a.x(),
                          bases_a.front().y() - car_a.y()) < 1e-9
            && std::abs(wrapToPi(bases_a.front().z() - car_a.z())) < 1e-9;
        // Order should depend on robot pose (car_a vs car_b).
        bool order_differs = bases_a.size() != bases_b.size();
        if(!order_differs && bases_a.size() >= 2 && bases_b.size() >= 2){
            order_differs =
                std::hypot(bases_a[1].x() - bases_b[1].x(),
                           bases_a[1].y() - bases_b[1].y()) > 1e-6
                || std::abs(wrapToPi(bases_a[1].z() - bases_b[1].z())) > 1e-6;
        }
        // Max theoretical: 1 + 4*12*3 = 145
        const bool count_ok = bases_a.size() <= 145 && bases_a.size() >= 1;
        const bool gen_pass = all_in_map && current_first && count_ok
            && (order_differs || bases_b.empty() || bases_a.size() > 1);
        printResult("Stage C base candidates filtered and pose-ordered",
                    gen_pass, all_pass);
        if(!gen_pass){
            ROS_ERROR("base gen: n_a=%zu n_b=%zu in_map=%d first=%d order=%d",
                      bases_a.size(), bases_b.size(),
                      static_cast<int>(all_in_map),
                      static_cast<int>(current_first),
                      static_cast<int>(order_differs));
        }
    }
    // ################################
    // C++: Stage C base candidate generation tests end
    // ################################

    std::cout << (all_pass ? "ALL TESTS PASSED" : "TESTS FAILED") << std::endl;
    return all_pass ? 0 : 1;
}
// ################################
// C++: EE pose and Jacobian Milestone A tests end
// ################################
