// ################################
// C++: CR10 FK vs URDF analytic reference begin
// ################################
#include <mm_config/mm_config.hpp>
#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <random>

using remani_planner::MMConfig;

static Eigen::Matrix3d urdfRpy(double r, double p, double y) {
  Eigen::Quaterniond q = Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ())
                       * Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY())
                       * Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX());
  return q.toRotationMatrix();
}

static Eigen::Matrix4d makeFixed(double x, double y, double z, double rr, double pp, double yy) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block(0, 0, 3, 3) = urdfRpy(rr, pp, yy);
  T(0, 3) = x; T(1, 3) = y; T(2, 3) = z;
  return T;
}

static Eigen::Matrix4d Rz(double th) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T(0, 0) = std::cos(th); T(0, 1) = -std::sin(th);
  T(1, 0) = std::sin(th); T(1, 1) =  std::cos(th);
  return T;
}

static double rotErrDeg(const Eigen::Matrix3d &A, const Eigen::Matrix3d &B) {
  Eigen::Matrix3d R = A.transpose() * B;
  double tr = std::max(-1.0, std::min(1.0, (R.trace() - 1.0) * 0.5));
  return std::acos(tr) * 180.0 / M_PI;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_cr10_fk_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // Ensure CR10 params if not loaded from launch
  if (!nh.hasParam("mm/manipulator_type")) {
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
    std::vector<double> cfg{0.1765, 0.607, 0.568, 0.191, 0.125, 0.1084};
    nh.setParam("mm/manipulator_config", cfg);
    std::vector<double> qmin{-3, -1.5, -2.8, -3, -3, -3};
    std::vector<double> qmax{3, 1.5, 2.8, 3, 3, 3};
    nh.setParam("mm/manipulator_min_pos", qmin);
    nh.setParam("mm/manipulator_max_pos", qmax);
    std::vector<double> mount{0.2462, 0.0, 0.1, 0.0, 0.0, 0.0};
    nh.setParam("mm/base_mani_fixed_joint_xyz_ypr", mount);
    nh.setParam("optimization/safe_margin", 0.05);
    nh.setParam("optimization/safe_margin_mani", 0.05);
    nh.setParam("optimization/self_safe_margin", 0.02);
  }

  MMConfig cfg;
  cfg.setParam(nh);

  if (cfg.getManipulatorType() != MMConfig::ManipulatorType::CR10) {
    ROS_ERROR("manipulator_type is not cr10");
    return 1;
  }

  Eigen::Matrix4d F[6];
  F[0] = makeFixed(0, 0, 0.1765, 0, 0, 0);
  F[1] = makeFixed(0, 0, 0, 1.5708, 1.5708, 0);
  F[2] = makeFixed(-0.607, 0, 0, 0, 0, 0);
  F[3] = makeFixed(-0.568, 0, 0.191, 0, 0, -1.5708);
  F[4] = makeFixed(0, -0.125, 0, 1.5708, 0, 0);
  F[5] = makeFixed(0, 0.1084, 0, -1.5708, 0, 0);

  Eigen::Matrix4d T_mount = cfg.getTq0();
  Eigen::Matrix4d T_mount_ref = makeFixed(0.2462, 0, 0.1, 0, 0, 0);
  double mount_pos_err = (T_mount.block(0, 3, 3, 1) - T_mount_ref.block(0, 3, 3, 1)).norm();
  double mount_rot_err = rotErrDeg(T_mount.block(0, 0, 3, 3), T_mount_ref.block(0, 0, 3, 3));
  ROS_INFO("T_q_0_ pos err=%.6e m, rot err=%.6e deg", mount_pos_err, mount_rot_err);
  if (mount_pos_err > 1e-6 || mount_rot_err > 1e-3) {
    ROS_ERROR("T_q_0_ does not match CR10 mount (YAML override bug?)");
    return 1;
  }

  std::mt19937 gen(0);
  std::uniform_real_distribution<double> u(0.0, 1.0);
  Eigen::VectorXd qmin(6), qmax(6);
  qmin << -3, -1.5, -2.8, -3, -3, -3;
  qmax << 3, 1.5, 2.8, 3, 3, 3;

  double max_pos = 0, max_rot = 0, max_grad = 0;
  const double eps = 1e-6;
  const int N = 100;

  for (int n = 0; n < N; ++n) {
    Eigen::VectorXd q(6);
    for (int i = 0; i < 6; ++i) q(i) = qmin(i) + (qmax(i) - qmin(i)) * u(gen);

    Eigen::Matrix4d T_ref = T_mount_ref;
    Eigen::Matrix4d T_mm = T_mount;
    for (int i = 0; i < 6; ++i) {
      Eigen::Matrix4d Ti, dTi;
      cfg.getAJointTran(i, q(i), Ti, dTi);
      T_mm = T_mm * Ti;
      T_ref = T_ref * (F[i] * Rz(q(i)));

      Eigen::Matrix4d Tp, Tm, d_unused;
      cfg.getAJointTran(i, q(i) + eps, Tp, d_unused);
      cfg.getAJointTran(i, q(i) - eps, Tm, d_unused);
      Eigen::Matrix4d Tfd = (Tp - Tm) / (2.0 * eps);
      double denom = std::max(dTi.norm(), 1e-12);
      max_grad = std::max(max_grad, (dTi - Tfd).norm() / denom);
    }

    max_pos = std::max(max_pos, (T_mm.block(0, 3, 3, 1) - T_ref.block(0, 3, 3, 1)).norm());
    max_rot = std::max(max_rot, rotErrDeg(T_mm.block(0, 0, 3, 3), T_ref.block(0, 0, 3, 3)));
  }

  ROS_INFO("samples=%d max_pos_err=%.6e m max_rot_err=%.6e deg max_grad_rel=%.6e",
           N, max_pos, max_rot, max_grad);

  // Car-mani self collision smoke: folded arm
  Eigen::VectorXd q_fold(6);
  q_fold << 0, -1.2, 2.0, 0, 0.8, 0;
  double min_dist = 0;
  bool self_hit = cfg.checkCarManiCollision(q_fold, true, min_dist);
  ROS_INFO("example folded config car-mani collision(safe)=%d min_dist=%.3f", (int)self_hit, min_dist);

  bool ok = (max_pos < 1e-3) && (max_rot < 0.1) && (max_grad < 1e-4);
  if (!ok) {
    ROS_ERROR("CR10 FK/grad validation FAILED");
    return 1;
  }
  ROS_INFO("CR10 FK/grad validation PASSED");
  return 0;
}
// ################################
// C++: CR10 FK vs URDF analytic reference end
// ################################
