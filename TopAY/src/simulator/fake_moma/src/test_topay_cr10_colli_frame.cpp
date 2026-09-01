// ################################
// C++: CR10 planning collision sphere frame diagnosis gate
// ################################
#include "fake_moma/moma_param.h"

#include <cmath>
#include <iomanip>
#include <iostream>

namespace
{
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

Eigen::VectorXd makeState(double x, double y, double yaw, const Eigen::VectorXd& q)
{
    Eigen::VectorXd state(3 + q.size());
    state << x, y, yaw, q;
    return state;
}

bool dumpAndVerifyPlanningTruth(const MomaParam& profile, const Eigen::VectorXd& state,
                                const char* label, double tol)
{
    const KinematicResult links = profile.getLinkTransformsCr10(state);
    const std::vector<Eigen::Vector4d> colli_pts = profile.getColliPtsCr10(state);

    std::cout << "\n=== CR10 collision frame report: " << label << " ===\n";
    std::cout << std::fixed << std::setprecision(6);
    double max_err = 0.0;
    for (size_t i = 0; i < profile.collision_proxies_.size(); ++i)
    {
        const CollisionSphere& proxy = profile.collision_proxies_[i];
        const Eigen::Matrix4d owner_T = profile.cr10OwnerLinkTransform(links, proxy.link_id);
        const Eigen::Vector4d local = (Eigen::Vector4d() << proxy.local_offset, 1.0).finished();
        const Eigen::Vector3d expected = (owner_T * local).head<3>();
        const Eigen::Vector3d actual = colli_pts[i].head<3>();
        const double err = (actual - expected).norm();
        max_err = std::max(max_err, err);

        std::cout << "idx=" << i
                  << " link_id=" << proxy.link_id
                  << " local_offset=" << proxy.local_offset.transpose()
                  << " planning=" << actual.transpose()
                  << " expected=" << expected.transpose()
                  << " r_obs=" << proxy.obstacle_radius
                  << " r_self=" << proxy.self_radius
                  << " err=" << err << "\n";
    }

    std::cout << "max planning truth err=" << max_err << "\n";
    if (max_err > tol)
    {
        std::cerr << "CR10 planning sphere numeric truth gate FAILED: " << label << "\n";
        return false;
    }
    return true;
}
}  // namespace

int main()
{
    const MomaParam profile = makeCr10Profile();
    const Eigen::VectorXd q_zero = Eigen::VectorXd::Zero(6);
    const Eigen::VectorXd q_nonzero = (Eigen::VectorXd(6) << 0.3, -0.4, 0.8, -0.5, 0.6, -0.2).finished();

    const Eigen::VectorXd state0 = makeState(0.0, 0.0, 0.0, q_zero);
    const Eigen::VectorXd state1 = makeState(1.0, -0.5, 0.6, q_nonzero);

    bool ok = true;
    ok = dumpAndVerifyPlanningTruth(profile, state0, "state0 (x=0,y=0,yaw=0,q=0)", 1e-9) && ok;
    ok = dumpAndVerifyPlanningTruth(profile, state1, "state1 (x=1,y=-0.5,yaw=0.6,q!=0)", 1e-9) && ok;

    std::cout << "\n=== Diagnosis notes ===\n";
    std::cout << "RViz /sphere uses obstacle_radius (0.10), not self_radius (0.045).\n";
    std::cout << "Planning spheres are in base z=0 frame; CAD uses visual root +0.275 (Task 2B overlay).\n";

    if (!ok)
    {
        return 1;
    }

    std::cout << "CR10 planning sphere numeric truth gate PASSED\n";
    return 0;
}
