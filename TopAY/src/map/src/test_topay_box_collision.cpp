// ################################
// C++: Ranger upper-box discrete + continuous GridMap collision gate (Case A/B/C/D)
// ################################
#include "map/grid_map.h"

#include <ros/ros.h>

#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

namespace
{
MomaParam makeCr10ProfileWithBoxObstacle()
{
    MomaParam profile;
    profile.robot_name = "ranger_cr10";
    profile.kinematics = KinematicsType::Cr10;
    profile.dof_num = 6;
    profile.chassis_height = 0.15;
    profile.chassis_colli_radius = 0.711;
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
    profile.box_obstacle_enabled_ = true;
    profile.box_obstacle_margin_ = 0.02;
    const std::array<double, 3> grid_x = {-0.45, 0.0, 0.45};
    const std::array<double, 3> grid_y = {-0.35, 0.0, 0.35};
    const std::array<double, 2> grid_z = {0.18, 0.38};
    for (double x : grid_x)
    {
        for (double y : grid_y)
        {
            for (double z : grid_z)
            {
                CollisionSphere sphere;
                sphere.local_offset << x, y, z;
                sphere.obstacle_radius = 0.18;
                profile.base_obstacle_proxies_.push_back(sphere);
            }
        }
    }
    profile.buildBaseObstacleProxies();
    return profile;
}

void stampObstacleSphere(nmoma_planner::GridMap& map,
                         const Eigen::Vector3d& center,
                         double radius,
                         double chassis_height,
                         std::vector<char>& occ_2d,
                         std::vector<char>& occ_3d)
{
    for (int ix = 0; ix < map.voxel_num(0); ++ix)
    {
        for (int iy = 0; iy < map.voxel_num(1); ++iy)
        {
            for (int iz = 0; iz < map.voxel_num(2); ++iz)
            {
                const Eigen::Vector3i id(ix, iy, iz);
                Eigen::Vector3d pos;
                map.indexToPos3d(id, pos);
                if ((pos - center).norm() > radius)
                {
                    continue;
                }
                occ_3d[map.toAddress3d(id)] = 1;
                if (pos.z() < chassis_height)
                {
                    occ_2d[map.toAddress2d(ix, iy)] = 1;
                }
            }
        }
    }
}

void loadObstacleAt(nmoma_planner::GridMap& map,
                    const Eigen::Vector3d& center,
                    double radius,
                    double chassis_height)
{
    std::vector<char> occ_2d(map.buffer_size_2d, 0);
    std::vector<char> occ_3d(map.buffer_size_3d, 0);
    stampObstacleSphere(map, center, radius, chassis_height, occ_2d, occ_3d);
    map.loadMap(occ_2d, occ_3d);
}

nmoma_planner::GridMap& initTestGridMap(nmoma_planner::GridMap& grid_map,
                                        const std::shared_ptr<const MomaParam>& profile)
{
    ros::NodeHandle nh("~");
    nh.setParam("grid_map/map_size_x", 4.0);
    nh.setParam("grid_map/map_size_y", 4.0);
    nh.setParam("grid_map/map_size_z", 2.0);
    nh.setParam("grid_map/resolution", 0.10);
    nh.setParam("agent/mode", std::string("planner"));
    nh.setParam("agent/fixed_sequence", false);
    nh.setParam("grid_map/use_rog", false);

    grid_map.init(nh);
    grid_map.setMomaParam(profile);
    return grid_map;
}
}  // namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_topay_box_collision", ros::init_options::AnonymousName);

    const auto profile = std::make_shared<const MomaParam>(makeCr10ProfileWithBoxObstacle());
    nmoma_planner::GridMap grid_map;
    initTestGridMap(grid_map, profile);
    const Eigen::VectorXd home = Eigen::VectorXd::Zero(9);

    // Case A — low obstacle inside chassis 2D disk
    loadObstacleAt(grid_map, Eigen::Vector3d(0.0, 0.0, 0.05), 0.08, profile->chassis_height);
    if (!grid_map.isWholeBodyCollision(home))
    {
        std::cerr << "Case A low-obstacle collision FAILED (expected true)" << std::endl;
        return 1;
    }
    std::cout << "Case A low-obstacle collision PASSED" << std::endl;

    // Case B — mid-high obstacle intersecting upper box envelope
    loadObstacleAt(grid_map, Eigen::Vector3d(0.0, 0.0, 0.28), 0.08, profile->chassis_height);
    if (!grid_map.isWholeBodyCollision(home))
    {
        std::cerr << "Case B upper-box obstacle collision FAILED (expected true)" << std::endl;
        return 1;
    }
    std::cout << "Case B upper-box obstacle collision PASSED" << std::endl;

    // Case C — obstacle above box top clearance
    loadObstacleAt(grid_map, Eigen::Vector3d(0.0, 0.0, 0.65), 0.08, profile->chassis_height);
    if (grid_map.isWholeBodyCollision(home))
    {
        std::cerr << "Case C above-box clearance FAILED (expected false)" << std::endl;
        return 1;
    }
    std::cout << "Case C above-box clearance PASSED" << std::endl;

    // ################################
    // C++: Case D — continuous trajectory obstacle-cost gate (midpoint only)
    // ################################
    // Side mid-height obstacle hits box envelope (+y spheres) but not CR10 arm chain.
    loadObstacleAt(grid_map, Eigen::Vector3d(0.0, 0.35, 0.28), 0.08, profile->chassis_height);
    const Eigen::VectorXd start = (Eigen::VectorXd(9) << -1.5, 0.0, 0.0, 0, 0, 0, 0, 0, 0).finished();
    const Eigen::VectorXd mid = (Eigen::VectorXd(9) << 0.0, 0.0, 0.0, 0, 0, 0, 0, 0, 0).finished();
    const Eigen::VectorXd end = (Eigen::VectorXd(9) << 1.5, 0.0, 0.0, 0, 0, 0, 0, 0, 0).finished();
    if (grid_map.isWholeBodyCollision(start) || grid_map.isWholeBodyCollision(end))
    {
        std::cerr << "Case D endpoints must remain collision-free" << std::endl;
        return 1;
    }
    if (!grid_map.isWholeBodyCollision(mid))
    {
        std::cerr << "Case D midpoint must be discrete-colliding with box envelope" << std::endl;
        return 1;
    }

    auto smoothL1Penalty = [](double x, double& cost, double& grad) {
        const double mu = 1e-4;
        if (x <= 0.0)
        {
            cost = 0.0;
            grad = 0.0;
            return 0.0;
        }
        if (x <= mu)
        {
            cost = x * x / (2.0 * mu);
            grad = x / mu;
            return cost;
        }
        cost = x - 0.5 * mu;
        grad = 1.0;
        return cost;
    };

    const double cost_scale = 10.0;
    const double mani_colli_weight = 100.0;
    double mid_cost = 0.0;
    std::vector<Eigen::Vector3d> base_pos_grads;
    for (const Eigen::Vector4d& pt : profile->getBaseObstaclePts(mid))
    {
        double sdf_value = 0.0;
        Eigen::Vector3d grad_pc = Eigen::Vector3d::Zero();
        grid_map.getDisWithGradI3d(pt.head(3), sdf_value, grad_pc);
        const double viola = pt(3) * cost_scale * 1.1 - sdf_value * cost_scale;
        double pena = 0.0;
        double pena_d = 0.0;
        Eigen::Vector3d grad_to_pos = Eigen::Vector3d::Zero();
        smoothL1Penalty(viola, pena, pena_d);
        if (viola > 0.0)
        {
            mid_cost += mani_colli_weight * pena;
            grad_to_pos = -mani_colli_weight * pena_d * grad_pc * cost_scale;
        }
        base_pos_grads.push_back(grad_to_pos);
    }
    if (mid_cost <= 0.0)
    {
        std::cerr << "Case D midpoint trajectory cost FAILED (expected > 0)" << std::endl;
        return 1;
    }

    const Eigen::VectorXd mid_grad = profile->getBaseObstacleGrads(mid, base_pos_grads);
    if (!std::isfinite(mid_grad(0)) || !std::isfinite(mid_grad(1)) || !std::isfinite(mid_grad(2)))
    {
        std::cerr << "Case D midpoint base grad not finite: " << mid_grad.head(3).transpose()
                  << std::endl;
        return 1;
    }
    // Obstacle at +y: expect nonzero ∂cost/∂y into optimizer; yaw FD in test_topay_cr10_fk
    if (std::abs(mid_grad(1)) < 1e-6)
    {
        std::cerr << "Case D expected nonzero y grad: " << mid_grad.head(3).transpose()
                  << std::endl;
        return 1;
    }
    std::cout << "Case D continuous trajectory obstacle-cost PASSED"
              << " mid_cost=" << mid_cost
              << " mid_base_grad=" << mid_grad.head(3).transpose() << std::endl;

    std::cout << "Ranger box discrete collision gate PASSED" << std::endl;
    return 0;
}
