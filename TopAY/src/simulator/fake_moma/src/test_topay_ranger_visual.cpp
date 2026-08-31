// ################################
// C++: Ranger visual-root / LiDAR / D435 numeric regression gate
// ################################
#include "fake_moma/moma_param.h"
#include "fake_moma/visual_transform_utils.h"

#include <cmath>
#include <iostream>
#include <string>

namespace
{
Eigen::Matrix3d urdfRpy(double roll, double pitch, double yaw)
{
    return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

Eigen::Matrix4d makeT(double x, double y, double z, double roll, double pitch, double yaw)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = urdfRpy(roll, pitch, yaw);
    T(0, 3) = x;
    T(1, 3) = y;
    T(2, 3) = z;
    return T;
}

MomaParam makeRangerVisualProfile()
{
    MomaParam profile;
    profile.robot_name = "ranger_cr10";
    profile.kinematics = KinematicsType::Cr10;
    profile.dof_num = 6;
    profile.relative_R = Eigen::Matrix3d::Identity();
    profile.relative_t << 0.2462, 0.0, 0.1;
    profile.visual_base_xyz << 0.0, 0.0, 0.275;
    profile.visual_base_rpy.setZero();
    profile.link_length.resize(6);
    profile.link_length << 0.1765, 0.607, 0.568, 0.191, 0.125, 0.1084;
    profile.joint_pos_limit_min.resize(6);
    profile.joint_pos_limit_max.resize(6);
    profile.joint_pos_limit_min << -3.0, -1.5, -2.8, -3.0, -3.0, -3.0;
    profile.joint_pos_limit_max << 3.0, 1.5, 2.8, 3.0, 3.0, 3.0;
    profile.obstacle_thickness = 0.10;
    profile.self_thickness = 0.045;
    profile.sphere_spacing_factor = 1.0;
    profile.chassis_ignore_link_ids = {0, 1};
    profile.ag95_spheres = {
        {Eigen::Vector3d(0.0, 0.0, 0.06), 0.05, 0.04, 0},
    };

    auto addBase = [&](const std::string& file, double x, double y, double z,
                       double r, double p, double yaw,
                       const Eigen::Vector3d& scale = Eigen::Vector3d::Ones()) {
        MeshPart part;
        part.role = MeshRole::Base;
        part.file = file;
        part.link_T_visual = makeT(x, y, z, r, p, yaw);
        part.scale = scale;
        profile.mesh_parts.push_back(part);
    };

    addBase("ranger_base", 0.0, 0.0, 0.0, 0, 0, 0);
    addBase("box", 0.0, 0.0, 0.09, 0, 0, 0);
    addBase("lidar", 0.52588, 0.0, 0.16587, 0, 0, 0);
    // ################################
    // C++: D435 = lidar_joint0 * d435_joint * d435_link_joint * visual origin
    // ################################
    const Eigen::Matrix4d d435_T =
        makeT(0.52588, 0, 0.16587, 0, 0, 0)
        * makeT(0.085, 0, 0.058, 0, 0, 0)
        * makeT(0.0106, 0.0175, 0.0125, 0, 0, 0)
        * makeT(0.0043, -0.0175, 0, M_PI / 2.0, 0, M_PI / 2.0);
    {
        MeshPart part;
        part.role = MeshRole::Base;
        part.file = "d435";
        part.link_T_visual = d435_T;
        profile.mesh_parts.push_back(part);
    }
    {
        MeshPart part;
        part.role = MeshRole::ArmBase;
        part.file = "cr10_base";
        part.link_T_visual.setIdentity();
        profile.mesh_parts.push_back(part);
    }
    {
        MeshPart part;
        part.role = MeshRole::Ag95;
        part.file = "ag95";
        part.link_T_visual.setIdentity();
        part.scale = Eigen::Vector3d(0.001, 0.001, 0.001);
        profile.mesh_parts.push_back(part);
    }

    profile.initCr10FixedTransforms();
    profile.finalizeVisualization();
    return profile;
}

bool near(double a, double b, double tol = 1e-9)
{
    return std::abs(a - b) <= tol;
}

bool checkZeroStateVisual(const MomaParam& profile)
{
    const Eigen::VectorXd state = Eigen::VectorXd::Zero(3 + static_cast<int>(profile.dof_num));
    const KinematicResult links = profile.getLinkTransforms(state);
    const std::vector<Eigen::VectorXd> poses = profile.getMeshPose(state);

    Eigen::Vector3d ranger_base_z = Eigen::Vector3d::Zero();
    Eigen::Vector3d cr10_base = Eigen::Vector3d::Zero();
    Eigen::Vector3d lidar = Eigen::Vector3d::Zero();
    Eigen::Vector3d d435 = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < profile.mesh_parts.size(); ++i)
    {
        const MeshPart& part = profile.mesh_parts[i];
        const Eigen::Vector3d p = poses[i].head(3);
        if (part.file == "ranger_base")
            ranger_base_z = p;
        else if (part.file == "cr10_base")
            cr10_base = p;
        else if (part.file == "lidar")
            lidar = p;
        else if (part.file == "d435")
            d435 = p;
    }

    const double visual_root_z = profile.visual_base_xyz.z();
    if (!near(ranger_base_z.z(), visual_root_z))
    {
        std::cerr << "ranger visual base z expected " << visual_root_z
                  << " got " << ranger_base_z.z() << "\n";
        return false;
    }
    if (!near(cr10_base.z() - ranger_base_z.z(), 0.1))
    {
        std::cerr << "visual CR10-base relative Z expected 0.1 got "
                  << (cr10_base.z() - ranger_base_z.z()) << "\n";
        return false;
    }
    if (!near(lidar.z(), visual_root_z + 0.16587)
        || !near(lidar.x(), 0.52588))
    {
        std::cerr << "lidar visual pose mismatch: " << lidar.transpose() << "\n";
        return false;
    }
    if (!near(d435.x(), 0.62578) || !near(d435.y(), 0.0)
        || !near(d435.z(), visual_root_z + 0.23637))
    {
        std::cerr << "d435 visual pose mismatch: " << d435.transpose() << "\n";
        return false;
    }

    // ################################
    // C++: Planning FK mount must stay at z=0.1 without visual root
    // ################################
    if (!near(links.arm_base_T(2, 3), 0.1) || !near(links.base_T(2, 3), 0.0))
    {
        std::cerr << "planning FK polluted by visual root\n";
        return false;
    }
    return true;
}

bool checkMotionRigid(const MomaParam& profile)
{
    Eigen::VectorXd state = Eigen::VectorXd::Zero(3 + static_cast<int>(profile.dof_num));
    state << 1.5, -0.7, 0.4, 0.2, -0.3, 0.1, 0.0, 0.0, 0.0;
    const std::vector<Eigen::VectorXd> poses = profile.getMeshPose(state);

    Eigen::Vector3d ranger;
    Eigen::Vector3d lidar;
    Eigen::Vector3d cr10;
    for (size_t i = 0; i < profile.mesh_parts.size(); ++i)
    {
        if (profile.mesh_parts[i].file == "ranger_base")
            ranger = poses[i].head(3);
        if (profile.mesh_parts[i].file == "lidar")
            lidar = poses[i].head(3);
        if (profile.mesh_parts[i].file == "cr10_base")
            cr10 = poses[i].head(3);
    }

    const Eigen::Matrix3d R =
        Eigen::AngleAxisd(state(2), Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d expected_lidar =
        R * Eigen::Vector3d(0.52588, 0.0, 0.16587 + 0.275) + Eigen::Vector3d(state(0), state(1), 0.0);
    // visual root is pure +Z translation, so world z = planning_z + 0.275 after yaw
    const Eigen::Vector3d expected_lidar2 =
        Eigen::Vector3d(state(0), state(1), 0.0)
        + R * Eigen::Vector3d(0.52588, 0.0, 0.16587)
        + Eigen::Vector3d(0.0, 0.0, 0.275);

    if ((lidar - expected_lidar2).norm() > 1e-9)
    {
        std::cerr << "motion lidar expected " << expected_lidar2.transpose()
                  << " got " << lidar.transpose() << "\n";
        return false;
    }
    const Eigen::Vector3d expected_cr10 =
        Eigen::Vector3d(state(0), state(1), 0.0)
        + R * Eigen::Vector3d(0.2462, 0.0, 0.1)
        + Eigen::Vector3d(0.0, 0.0, 0.275);
    if ((cr10 - expected_cr10).norm() > 1e-9)
    {
        std::cerr << "motion CR10 expected " << expected_cr10.transpose()
                  << " got " << cr10.transpose() << "\n";
        return false;
    }
    (void)ranger;
    (void)expected_lidar;
    return true;
}
}  // namespace

int main()
{
    const MomaParam profile = makeRangerVisualProfile();
    if (!checkZeroStateVisual(profile))
    {
        std::cerr << "Ranger visual zero-state regression FAILED\n";
        return 1;
    }
    if (!checkMotionRigid(profile))
    {
        std::cerr << "Ranger visual motion regression FAILED\n";
        return 1;
    }
    std::cout << "Ranger visual-root / LiDAR / D435 regression PASSED\n";
    return 0;
}
