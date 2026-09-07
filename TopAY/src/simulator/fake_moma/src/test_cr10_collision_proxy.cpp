// ################################
// C++: CR10 FK/EE-grad + collision proxy transform + collision gradient FD
// ################################
#include "fake_moma/moma_param.h"

#include <ros/package.h>
#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <random>
#include <string>

namespace
{
bool loadProductionProfile(MomaParam& profile)
{
    // ################################
    // C++: Load production robot_ranger_cr10.yaml onto param server (no hardcoded geometry)
    // ################################
    const std::string yaml =
        ros::package::getPath("planner") + "/params/robot_ranger_cr10.yaml";
    const std::string cmd = "rosparam load " + yaml;
    if (std::system(cmd.c_str()) != 0)
    {
        std::cerr << "rosparam load failed: " << yaml << std::endl;
        return false;
    }
    profile = MomaParam::fromRos(ros::NodeHandle());
    return true;
}

Eigen::Matrix3d urdfRpy(double roll, double pitch, double yaw)
{
    const Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                               * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                               * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    return q.toRotationMatrix();
}

Eigen::Matrix4d makeFixed(double x, double y, double z, double roll, double pitch, double yaw)
{
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = urdfRpy(roll, pitch, yaw);
    transform(0, 3) = x;
    transform(1, 3) = y;
    transform(2, 3) = z;
    return transform;
}

Eigen::Matrix4d rz(double theta)
{
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform(0, 0) = std::cos(theta);
    transform(0, 1) = -std::sin(theta);
    transform(1, 0) = std::sin(theta);
    transform(1, 1) = std::cos(theta);
    return transform;
}

double rotErrDeg(const Eigen::Matrix3d& left, const Eigen::Matrix3d& right)
{
    const Eigen::Matrix3d delta = left.transpose() * right;
    const double trace = std::max(-1.0, std::min(1.0, (delta.trace() - 1.0) * 0.5));
    return std::acos(trace) * 180.0 / M_PI;
}

double scalarFk(const MomaParam& profile, const Eigen::VectorXd& state,
                const Eigen::VectorXd& ee_grad)
{
    const Eigen::VectorXd fk_pose = profile.getFKPose(state);
    return ee_grad.head(3).dot(fk_pose.head(3))
           + ee_grad.segment(3, 3).dot(fk_pose.segment(3, 3))
           + ee_grad.tail(3).dot(fk_pose.tail(3));
}

double scalarSphere(const MomaParam& profile, const Eigen::VectorXd& state,
                    size_t sphere_idx, const Eigen::Vector3d& direction)
{
    const std::vector<Eigen::Vector4d> pts = profile.getColliPts(state);
    return direction.dot(pts[sphere_idx].head<3>());
}

bool verifyPlanningTransform(const MomaParam& profile, const Eigen::VectorXd& state,
                             double tol)
{
    const KinematicResult links = profile.getLinkTransformsCr10(state);
    const std::vector<Eigen::Vector4d> colli_pts = profile.getColliPtsCr10(state);
    if (colli_pts.size() != profile.collision_proxies_.size())
    {
        std::cerr << "collision proxy count mismatch\n";
        return false;
    }
    double max_err = 0.0;
    for (size_t i = 0; i < profile.collision_proxies_.size(); ++i)
    {
        const CollisionSphere& proxy = profile.collision_proxies_[i];
        const Eigen::Matrix4d owner_T = profile.cr10OwnerLinkTransform(links, proxy.link_id);
        const Eigen::Vector4d local = (Eigen::Vector4d() << proxy.local_offset, 1.0).finished();
        const Eigen::Vector3d expected = (owner_T * local).head<3>();
        max_err = std::max(max_err, (colli_pts[i].head<3>() - expected).norm());
    }
    if (max_err > tol)
    {
        std::cerr << "CR10 collision proxy transform FAILED max_err=" << max_err << "\n";
        return false;
    }
    return true;
}
}  // namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_cr10_collision_proxy", ros::init_options::AnonymousName);
    MomaParam profile;
    if (!loadProductionProfile(profile))
    {
        return 1;
    }
    if (profile.kinematics != KinematicsType::Cr10 || profile.collision_proxies_.empty())
    {
        std::cerr << "expected CR10 profile with collision_proxies_\n";
        return 1;
    }

    // ################################
    // C++: FK / EE-grad FD (merged from test_topay_cr10_fk)
    // ################################
    const Eigen::Matrix4d mount_ref = makeFixed(
        profile.relative_t(0), profile.relative_t(1), profile.relative_t(2), 0.0, 0.0, 0.0);
    const KinematicResult mount_check = profile.getLinkTransforms(Eigen::VectorXd::Zero(9));
    if ((mount_check.arm_base_T.block<3, 1>(0, 3) - mount_ref.block<3, 1>(0, 3)).norm() > 1e-6)
    {
        std::cerr << "mount mismatch vs production relative_t\n";
        return 1;
    }

    Eigen::Matrix4d fixed[6];
    fixed[0] = makeFixed(0.0, 0.0, profile.link_length(0), 0.0, 0.0, 0.0);
    fixed[1] = makeFixed(0.0, 0.0, 0.0, 1.5708, 1.5708, 0.0);
    fixed[2] = makeFixed(-profile.link_length(1), 0.0, 0.0, 0.0, 0.0, 0.0);
    fixed[3] = makeFixed(-profile.link_length(2), 0.0, profile.link_length(3), 0.0, 0.0, -1.5708);
    fixed[4] = makeFixed(0.0, -profile.link_length(4), 0.0, 1.5708, 0.0, 0.0);
    fixed[5] = makeFixed(0.0, profile.link_length(5), 0.0, -1.5708, 0.0, 0.0);

    std::mt19937 generator(0);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    std::uniform_real_distribution<double> grad_unit(-1.0, 1.0);
    const double fd_eps = 1e-6;
    double max_pos_err = 0.0;
    double max_rot_err = 0.0;
    double max_grad_err = 0.0;
    const int sample_count = 100;
    for (int sample = 0; sample < sample_count; ++sample)
    {
        Eigen::VectorXd state = Eigen::VectorXd::Zero(9);
        for (int joint = 0; joint < 6; ++joint)
        {
            state(3 + joint) = profile.joint_pos_limit_min(joint)
                + (profile.joint_pos_limit_max(joint) - profile.joint_pos_limit_min(joint))
                    * unit(generator);
        }
        Eigen::Matrix4d reference = mount_ref;
        const KinematicResult transforms = profile.getLinkTransforms(state);
        for (int joint = 0; joint < 6; ++joint)
        {
            reference = reference * fixed[joint] * rz(state(3 + joint));
        }
        max_pos_err = std::max(
            max_pos_err,
            (transforms.ee_T.block<3, 1>(0, 3) - reference.block<3, 1>(0, 3)).norm());
        max_rot_err = std::max(
            max_rot_err,
            rotErrDeg(transforms.ee_T.block<3, 3>(0, 0), reference.block<3, 3>(0, 0)));

        Eigen::VectorXd ee_grad = Eigen::VectorXd::Zero(9);
        for (int entry = 0; entry < 9; ++entry)
        {
            ee_grad(entry) = grad_unit(generator);
        }
        const Eigen::VectorXd analytic_grad = profile.getEEGrads(state, ee_grad);
        for (int joint = 0; joint < 6; ++joint)
        {
            Eigen::VectorXd plus = state;
            Eigen::VectorXd minus = state;
            plus(3 + joint) += fd_eps;
            minus(3 + joint) -= fd_eps;
            const double fd_grad = (scalarFk(profile, plus, ee_grad)
                                    - scalarFk(profile, minus, ee_grad))
                                   / (2.0 * fd_eps);
            const double denom = std::max(std::abs(fd_grad), 1e-12);
            max_grad_err = std::max(max_grad_err, std::abs(analytic_grad(3 + joint) - fd_grad) / denom);
        }
    }
    if (!(max_pos_err < 1e-3 && max_rot_err < 0.1 && max_grad_err < 1e-4))
    {
        std::cerr << "CR10 FK/EE-grad validation FAILED\n";
        return 1;
    }
    std::cout << "CR10 FK/EE-grad validation PASSED\n";

    // ################################
    // C++: Planning collision proxy transform (migrated from colli_frame, no markers)
    // ################################
    Eigen::VectorXd q0 = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd q1(6);
    q1 << 0.2, -0.3, 0.4, -0.5, 0.1, 0.0;
    Eigen::VectorXd state0(9);
    state0 << 0.0, 0.0, 0.0, q0;
    Eigen::VectorXd state1(9);
    state1 << 0.5, -0.4, 0.7, q1;
    if (!verifyPlanningTransform(profile, state0, 1e-9)
        || !verifyPlanningTransform(profile, state1, 1e-9))
    {
        return 1;
    }
    std::cout << "CR10 collision proxy transform PASSED\n";

    // ################################
    // C++: getColliGradsCr10 FD (arm joints + base x/y/yaw)
    // ################################
    double max_colli_grad_err = 0.0;
    const int colli_samples = 100;
    for (int sample = 0; sample < colli_samples; ++sample)
    {
        Eigen::VectorXd state = Eigen::VectorXd::Zero(9);
        state(0) = unit(generator) - 0.5;
        state(1) = unit(generator) - 0.5;
        state(2) = (unit(generator) - 0.5) * 2.0;
        for (int joint = 0; joint < 6; ++joint)
        {
            state(3 + joint) = profile.joint_pos_limit_min(joint)
                + (profile.joint_pos_limit_max(joint) - profile.joint_pos_limit_min(joint))
                    * unit(generator);
        }
        const size_t sphere_idx = static_cast<size_t>(sample % profile.collision_proxies_.size());
        Eigen::Vector3d direction(grad_unit(generator), grad_unit(generator), grad_unit(generator));
        if (direction.norm() < 1e-6)
        {
            direction = Eigen::Vector3d::UnitX();
        }
        direction.normalize();
        std::vector<Eigen::Vector3d> pos_grads(profile.collision_proxies_.size(),
                                                 Eigen::Vector3d::Zero());
        pos_grads[sphere_idx] = direction;
        const Eigen::VectorXd analytic_grad = profile.getColliGrads(state, pos_grads);
        for (int dof = 0; dof < 9; ++dof)
        {
            Eigen::VectorXd plus = state;
            Eigen::VectorXd minus = state;
            plus(dof) += fd_eps;
            minus(dof) -= fd_eps;
            const double fd_grad = (scalarSphere(profile, plus, sphere_idx, direction)
                                    - scalarSphere(profile, minus, sphere_idx, direction))
                                   / (2.0 * fd_eps);
            const double denom = std::max(std::abs(fd_grad), 1e-12);
            max_colli_grad_err = std::max(
                max_colli_grad_err,
                std::abs(analytic_grad(dof) - fd_grad) / denom);
        }
    }
    if (max_colli_grad_err >= 1e-4)
    {
        std::cerr << "CR10 collision gradient FD FAILED max_rel_err="
                  << max_colli_grad_err << "\n";
        return 1;
    }
    std::cout << "CR10 collision gradient FD PASSED\n";
    return 0;
}
