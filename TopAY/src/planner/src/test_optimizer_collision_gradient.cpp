// ################################
// C++: Production MomaTrajOpt eeCostCallback base x/y/yaw FD (linked moma_traj_opt.cpp only)
// ################################
#include "map/grid_map.h"
#include "planner/moma_traj_opt.h"

#include <ros/package.h>
#include <ros/ros.h>

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace
{
std::shared_ptr<const MomaParam> loadProductionProfile()
{
    const std::string yaml =
        ros::package::getPath("planner") + "/params/robot_ranger_cr10.yaml";
    if (std::system(("rosparam load " + yaml).c_str()) != 0)
    {
        throw std::runtime_error("rosparam load failed: " + yaml);
    }
    return std::make_shared<const MomaParam>(MomaParam::fromRos(ros::NodeHandle()));
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

Eigen::VectorXd encodeVq(nmoma_planner::MomaTrajOpt& opt,
                         const MomaParam& profile,
                         const Eigen::VectorXd& moma_pos)
{
    Eigen::VectorXd x = Eigen::VectorXd::Zero(moma_pos.size());
    x.head(3) = moma_pos.head(3);
    for (int i = 0; i < profile.dof_num; ++i)
    {
        x(i + 3) = opt.invSigmoidC2(moma_pos(i + 3), profile.joint_pos_limit_max(i));
    }
    return x;
}
}  // namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_optimizer_collision_gradient",
              ros::init_options::AnonymousName);

    std::shared_ptr<const MomaParam> profile;
    try
    {
        profile = loadProductionProfile();
    }
    catch (const std::exception& ex)
    {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
    if (!profile->box_obstacle_enabled_ || profile->base_obstacle_proxies_.empty())
    {
        std::cerr << "production YAML must enable box_obstacle\n";
        return 1;
    }

    ros::NodeHandle nh("~");
    // ################################
    // C++: Finer ESDF grid so trilinear FD stays in a smooth neighborhood
    // ################################
    nh.setParam("grid_map/map_size_x", 4.0);
    nh.setParam("grid_map/map_size_y", 4.0);
    nh.setParam("grid_map/map_size_z", 2.0);
    nh.setParam("grid_map/resolution", 0.05);
    nh.setParam("agent/mode", std::string("planner"));
    nh.setParam("agent/fixed_sequence", false);
    nh.setParam("grid_map/use_rog", false);

    auto grid_map = std::make_shared<nmoma_planner::GridMap>();
    grid_map->init(nh);
    grid_map->setMomaParam(profile);

    nmoma_planner::MomaTrajOpt opt(grid_map);
    opt.setMomaParam(profile);
    // ################################
    // C++: eeCostCallback only needs relu_mu for smoothL1 — skip full init()
    // ################################
    opt.opt_param.relu_mu = 1.0e-3;

    // Pick production sphere with nonzero local xy (required for yaw FD signal)
    Eigen::Vector3d side_local = Eigen::Vector3d::Zero();
    for (const CollisionSphere& s : profile->base_obstacle_proxies_)
    {
        if (std::abs(s.local_offset.y()) > 1e-3)
        {
            side_local = s.local_offset;
            break;
        }
    }
    if (side_local.head<2>().norm() < 1e-6)
    {
        std::cerr << "production box_obstacle needs a sphere with nonzero local x/y\n";
        return 1;
    }

    // ################################
    // C++: FD step << resolution (0.05 m); keep sample neighborhood in one ESDF cell
    // ################################
    const double fd_eps = 5e-4;
    double max_rel_err = 0.0;
    const std::array<double, 2> yaw_states = {0.0, 0.7};
    // Off-grid base xy avoids yaw=0 lattice singularity in trilinear ESDF
    const double base_x = 0.041;
    const double base_y = -0.027;
    for (const double yaw : yaw_states)
    {
        // ################################
        // C++: Obstacle near rotated box sphere (partial overlap + 2D slope)
        // ################################
        const double c = std::cos(yaw);
        const double s = std::sin(yaw);
        const Eigen::Vector3d sphere_world(
            base_x + c * side_local.x() - s * side_local.y(),
            base_y + s * side_local.x() + c * side_local.y(),
            side_local.z());
        const Eigen::Vector3d obs_center = sphere_world + Eigen::Vector3d(0.07, 0.05, 0.0);
        loadObstacleAt(*grid_map, obs_center, 0.10, profile->chassis_height);

        Eigen::VectorXd moma_pos = Eigen::VectorXd::Zero(3 + profile->dof_num);
        moma_pos(0) = base_x;
        moma_pos(1) = base_y;
        moma_pos(2) = yaw;
        // Cancel EE tracking term so cost/grad dominate from collision path
        opt.setEEPose(profile->getFKPose(moma_pos));
        Eigen::VectorXd x = encodeVq(opt, *profile, moma_pos);
        Eigen::VectorXd analytic_grad = Eigen::VectorXd::Zero(x.size());
        const double cost = nmoma_planner::MomaTrajOpt::eeCostCallback(&opt, x, analytic_grad);
        if (!(cost > 0.0))
        {
            std::cerr << "Continuous trajectory collision cost FAILED (expected > 0) yaw="
                      << yaw << " cost=" << cost << "\n";
            return 1;
        }

        for (int dof = 0; dof < 3; ++dof)
        {
            Eigen::VectorXd plus = x;
            Eigen::VectorXd minus = x;
            plus(dof) += fd_eps;
            minus(dof) -= fd_eps;
            Eigen::VectorXd g_plus = Eigen::VectorXd::Zero(x.size());
            Eigen::VectorXd g_minus = Eigen::VectorXd::Zero(x.size());
            const double fd = (nmoma_planner::MomaTrajOpt::eeCostCallback(&opt, plus, g_plus)
                               - nmoma_planner::MomaTrajOpt::eeCostCallback(&opt, minus, g_minus))
                              / (2.0 * fd_eps);
            const double denom = std::max(
                std::max(std::abs(analytic_grad(dof)), std::abs(fd)), 1e-6);
            const double abs_err = std::abs(analytic_grad(dof) - fd);
            max_rel_err = std::max(max_rel_err, abs_err / denom);
        }
        std::cout << "yaw=" << yaw << " cost=" << cost
                  << " base_grad=" << analytic_grad.head(3).transpose() << "\n";
    }

    if (max_rel_err >= 1e-4)
    {
        std::cerr << "Base obstacle optimizer gradient FAILED max_rel_err="
                  << max_rel_err << "\n";
        return 1;
    }
    std::cout << "Base obstacle optimizer gradient PASSED max_rel_err=" << max_rel_err << "\n";
    std::cout << "Continuous trajectory collision cost PASSED\n";
    return 0;
}
