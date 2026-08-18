// ################################
// C++: CR10 FK and EE-gradient validation gates begin
// ################################
#include "fake_moma/moma_param.h"

#include <cmath>
#include <iostream>
#include <random>
#include <algorithm>

namespace
{
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

MomaParam makeCr10Profile()
{
    MomaParam profile;
    profile.robot_name = "ranger_cr10";
    profile.kinematics = KinematicsType::Cr10;
    profile.dof_num = 6;
    profile.relative_R = Eigen::Matrix3d::Identity();
    profile.relative_t << 0.2462, 0.0, 0.1;
    profile.obstacle_thickness = 0.10;
    profile.self_thickness = 0.045;
    profile.sphere_spacing_factor = 1.0;
    profile.chassis_ignore_link_ids = {0, 1};
    profile.link_length.resize(6);
    profile.link_length << 0.1765, 0.607, 0.568, 0.191, 0.125, 0.1084;
    profile.joint_pos_limit_min.resize(6);
    profile.joint_pos_limit_max.resize(6);
    profile.joint_pos_limit_min << -3.0, -1.5, -2.8, -3.0, -3.0, -3.0;
    profile.joint_pos_limit_max << 3.0, 1.5, 2.8, 3.0, 3.0, 3.0;
    profile.ag95_spheres = {
        {Eigen::Vector3d(0.0, 0.0, 0.06), 0.05, 0.04, 0},
        {Eigen::Vector3d(0.04, 0.03, 0.10), 0.03, 0.025, 0},
        {Eigen::Vector3d(0.04, -0.03, 0.10), 0.03, 0.025, 0},
    };
    profile.initCr10FixedTransforms();
    profile.buildCollisionProxies();
    profile.buildCollisionIgnoreMatrix();
    return profile;
}

double scalarSphere(const MomaParam& profile, const Eigen::VectorXd& state,
                    size_t sphere_idx, const Eigen::Vector3d& direction)
{
    const std::vector<Eigen::Vector4d> colli_pts = profile.getColliPts(state);
    return direction.dot(colli_pts[sphere_idx].head(3));
}

bool checkLongLinkCoverage(const MomaParam& profile, double max_gap)
{
    bool found_long_link = false;
    for (size_t joint = 0; joint < profile.dof_num; ++joint)
    {
        const Eigen::Vector3d segment = profile.cr10_joint_fixed_[joint].block<3, 1>(0, 3);
        const double segment_length = segment.norm();
        if (segment_length < 0.4)
        {
            continue;
        }

        found_long_link = true;
        const int link_id = static_cast<int>(1 + joint);
        const Eigen::Vector3d mid = 0.5 * segment;
        double best = 1e9;
        for (const CollisionSphere& proxy : profile.collision_proxies_)
        {
            if (proxy.link_id != link_id)
            {
                continue;
            }
            best = std::min(best, (proxy.local_offset - mid).norm());
        }
        if (best > max_gap)
        {
            return false;
        }
    }
    return found_long_link;
}

bool gridMapStyleSelfCollision(const MomaParam& profile, const Eigen::VectorXd& state)
{
    const std::vector<Eigen::Vector4d> colli_pts = profile.getColliPts(state);
    for (size_t i = 0; i < colli_pts.size(); ++i)
    {
        const double self_radius_i = profile.getColliSelfRadius(i);
        for (size_t j = i + 1; j < colli_pts.size(); ++j)
        {
            const double dist = (colli_pts[i].head(3) - colli_pts[j].head(3)).norm();
            const double self_sum = self_radius_i + profile.getColliSelfRadius(j);
            if (dist < self_sum && profile.collision_matrix(static_cast<Eigen::Index>(i),
                                                             static_cast<Eigen::Index>(j)) == -1)
            {
                return true;
            }
        }
    }
    return false;
}

double scalarFk(const MomaParam& profile, const Eigen::VectorXd& state,
                const Eigen::VectorXd& ee_grad)
{
    const Eigen::VectorXd fk_pose = profile.getFKPose(state);
    return ee_grad.head(3).dot(fk_pose.head(3))
           + ee_grad.segment(3, 3).dot(fk_pose.segment(3, 3))
           + ee_grad.tail(3).dot(fk_pose.tail(3));
}
}  // namespace

int main()
{
    const MomaParam profile = makeCr10Profile();

    const Eigen::Matrix4d mount_ref = makeFixed(0.2462, 0.0, 0.1, 0.0, 0.0, 0.0);
    const KinematicResult mount_check = profile.getLinkTransforms(Eigen::VectorXd::Zero(9));
    const double mount_pos_err =
        (mount_check.arm_base_T.block<3, 1>(0, 3) - mount_ref.block<3, 1>(0, 3)).norm();
    const double mount_rot_err =
        rotErrDeg(mount_check.arm_base_T.block<3, 3>(0, 0), mount_ref.block<3, 3>(0, 0));
    if (mount_pos_err > 1e-6 || mount_rot_err > 1e-3)
    {
        std::cerr << "mount mismatch pos=" << mount_pos_err << " rot_deg=" << mount_rot_err
                  << std::endl;
        return 1;
    }

    Eigen::Matrix4d fixed[6];
    fixed[0] = makeFixed(0.0, 0.0, 0.1765, 0.0, 0.0, 0.0);
    fixed[1] = makeFixed(0.0, 0.0, 0.0, 1.5708, 1.5708, 0.0);
    fixed[2] = makeFixed(-0.607, 0.0, 0.0, 0.0, 0.0, 0.0);
    fixed[3] = makeFixed(-0.568, 0.0, 0.191, 0.0, 0.0, -1.5708);
    fixed[4] = makeFixed(0.0, -0.125, 0.0, 1.5708, 0.0, 0.0);
    fixed[5] = makeFixed(0.0, 0.1084, 0.0, -1.5708, 0.0, 0.0);

    std::mt19937 generator(0);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    std::uniform_real_distribution<double> grad_unit(-1.0, 1.0);

    double max_pos_err = 0.0;
    double max_rot_err = 0.0;
    double max_grad_err = 0.0;
    const double fd_eps = 1e-6;
    const int sample_count = 100;

    for (int sample = 0; sample < sample_count; ++sample)
    {
        Eigen::VectorXd state = Eigen::VectorXd::Zero(9);
        for (int joint = 0; joint < 6; ++joint)
        {
            const double lower = profile.joint_pos_limit_min(joint);
            const double upper = profile.joint_pos_limit_max(joint);
            state(3 + joint) = lower + (upper - lower) * unit(generator);
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

    std::cout << "samples=" << sample_count << " max_pos_err=" << max_pos_err
              << " max_rot_err_deg=" << max_rot_err << " max_grad_rel_err=" << max_grad_err
              << std::endl;

    const bool ok = (max_pos_err < 1e-3) && (max_rot_err < 0.1) && (max_grad_err < 1e-4);
    if (!ok)
    {
        std::cerr << "CR10 FK/EE-grad validation FAILED" << std::endl;
        return 1;
    }

    std::cout << "CR10 FK/EE-grad validation PASSED" << std::endl;

    if (!checkLongLinkCoverage(profile, 0.05))
    {
        std::cerr << "CR10 long-link collision coverage FAILED" << std::endl;
        return 1;
    }

    double max_colli_grad_err = 0.0;
    const int colli_samples = 100;
    for (int sample = 0; sample < colli_samples; ++sample)
    {
        Eigen::VectorXd state = Eigen::VectorXd::Zero(9);
        for (int joint = 0; joint < 6; ++joint)
        {
            const double lower = profile.joint_pos_limit_min(joint);
            const double upper = profile.joint_pos_limit_max(joint);
            state(3 + joint) = lower + (upper - lower) * unit(generator);
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

    std::cout << "colli_samples=" << colli_samples
              << " max_colli_grad_rel_err=" << max_colli_grad_err << std::endl;
    if (max_colli_grad_err >= 1e-4)
    {
        std::cerr << "CR10 collision-gradient validation FAILED" << std::endl;
        return 1;
    }
    std::cout << "CR10 collision-gradient validation PASSED" << std::endl;

    Eigen::VectorXi collision_link;
    const Eigen::VectorXd home_state = Eigen::VectorXd::Zero(9);
    if (profile.isSelfCollision(home_state, collision_link)
        || gridMapStyleSelfCollision(profile, home_state))
    {
        std::cerr << "CR10 home pose self-collision regression FAILED" << std::endl;
        return 1;
    }

    Eigen::VectorXd folded_state = Eigen::VectorXd::Zero(9);
    folded_state << 0.0, 0.0, 0.0, 0.0, -1.5, -2.8, -3.0, 0.0, 0.0;
    if (!profile.isSelfCollision(folded_state, collision_link)
        || !gridMapStyleSelfCollision(profile, folded_state))
    {
        std::cerr << "CR10 folded unsafe pose regression FAILED" << std::endl;
        return 1;
    }
    std::cout << "CR10 collision pose regression PASSED" << std::endl;

    return 0;
}
// ################################
// C++: CR10 FK and EE-gradient validation gates end
// ################################
