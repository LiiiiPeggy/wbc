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
    profile.visual_base_xyz << 0.0, 0.0, 0.275;
    profile.visual_base_rpy.setZero();
    profile.visual_root_T_.setIdentity();
    profile.visual_root_T_.block<3, 1>(0, 3) = profile.visual_base_xyz;
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

bool verifyVisualOverlay(const MomaParam& profile, const Eigen::VectorXd& state,
                         const char* label, double tol)
{
    const KinematicResult links = profile.getLinkTransformsCr10(state);
    const visualization_msgs::MarkerArray visual_markers = profile.getColliVisualMarkerArray(state);

    if (visual_markers.markers.size() != profile.collision_proxies_.size())
    {
        std::cerr << "visual marker count mismatch for " << label << "\n";
        return false;
    }

    std::cout << "\n=== CR10 visual collision overlay report: " << label << " ===\n";
    double max_err = 0.0;
    for (size_t i = 0; i < profile.collision_proxies_.size(); ++i)
    {
        const CollisionSphere& proxy = profile.collision_proxies_[i];
        const Eigen::Matrix4d owner_T = profile.cr10OwnerLinkTransform(links, proxy.link_id);
        const Eigen::Matrix4d visual_owner_T = profile.applyVisualRoot(links.base_T, owner_T);
        const Eigen::Vector4d local = (Eigen::Vector4d() << proxy.local_offset, 1.0).finished();
        const Eigen::Vector3d expected = (visual_owner_T * local).head<3>();
        const Eigen::Vector3d actual(visual_markers.markers[i].pose.position.x,
                                     visual_markers.markers[i].pose.position.y,
                                     visual_markers.markers[i].pose.position.z);
        const double err = (actual - expected).norm();
        max_err = std::max(max_err, err);

        std::cout << "idx=" << i
                  << " expected_visual=" << expected.transpose()
                  << " actual_visual=" << actual.transpose()
                  << " err=" << err << "\n";
    }

    std::cout << "max visual overlay err=" << max_err << "\n";
    if (max_err > tol)
    {
        std::cerr << "CR10 visual collision overlay gate FAILED: " << label << "\n";
        return false;
    }
    return true;
}

bool verifyCr10CylinderExactChain(const MomaParam& profile)
{
    Eigen::VectorXd state = Eigen::VectorXd::Zero(9);
    state(3) = 0.3;
    state(4) = -0.4;

    const visualization_msgs::MarkerArray cylinders = profile.getColliCylinderArray(state);
    if (cylinders.markers.empty())
    {
        std::cerr << "CR10 cylinder debug visualization FAILED: empty marker array\n";
        return false;
    }

    const KinematicResult links = profile.getLinkTransformsCr10(state);
    size_t marker_idx = 1;
    double max_err = 0.0;
    for (size_t joint = 0; joint < profile.dof_num; ++joint)
    {
        const Eigen::Vector3d segment = profile.cr10_joint_fixed_[joint].block<3, 1>(0, 3);
        if (segment.norm() <= 1e-6)
        {
            continue;
        }
        if (marker_idx >= cylinders.markers.size())
        {
            std::cerr << "CR10 cylinder debug visualization FAILED: missing link marker\n";
            return false;
        }

        const int link_id = static_cast<int>(1 + joint);
        const Eigen::Matrix4d link_T = profile.cr10OwnerLinkTransform(links, link_id);
        const Eigen::Vector4d local_mid =
            (Eigen::Vector4d() << 0.5 * segment.x(), 0.5 * segment.y(), 0.5 * segment.z(), 1.0)
                .finished();
        const Eigen::Vector3d expected_center = (link_T * local_mid).head<3>();
        const Eigen::Vector3d actual(cylinders.markers[marker_idx].pose.position.x,
                                     cylinders.markers[marker_idx].pose.position.y,
                                     cylinders.markers[marker_idx].pose.position.z);
        max_err = std::max(max_err, (actual - expected_center).norm());
        if (std::abs(cylinders.markers[marker_idx].scale.z - segment.norm()) > 1e-6)
        {
            std::cerr << "CR10 cylinder debug visualization FAILED: segment length mismatch\n";
            return false;
        }
        ++marker_idx;
    }

    if (max_err > 1e-6)
    {
        std::cerr << "CR10 cylinder debug visualization FAILED: max center err=" << max_err << "\n";
        return false;
    }

    std::cout << "CR10 cylinder exact-chain debug visualization PASSED (markers="
              << cylinders.markers.size() << ", max_center_err=" << max_err << ")\n";
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
    ok = verifyVisualOverlay(profile, state0, "state0 overlay", 1e-6) && ok;
    ok = verifyVisualOverlay(profile, state1, "state1 overlay", 1e-6) && ok;
    ok = verifyCr10CylinderExactChain(profile) && ok;

    std::cout << "\n=== Diagnosis notes ===\n";
    std::cout << "RViz /sphere uses obstacle_radius (0.10), not self_radius (0.045).\n";
    std::cout << "Planning spheres are in base z=0 frame; CAD uses visual root +0.275 (Task 2B overlay).\n";

    if (!ok)
    {
        return 1;
    }

    std::cout << "CR10 planning sphere numeric truth gate PASSED\n";
    std::cout << "CR10 visual collision overlay exact local-offset transform gate PASSED\n";
    return 0;
}
