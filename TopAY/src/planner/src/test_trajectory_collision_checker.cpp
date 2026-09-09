// ################################
// C++: Planner-layer whole-body trajectory hard-validation Cases A/B/C
// ################################
#include "map/grid_map.h"
#include "planner/moma_traj_opt.h"
#include "planner/trajectory_collision_checker.h"
#include "utils/minco.hpp"

#include <ros/package.h>
#include <ros/ros.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
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

void loadEmptyMap(nmoma_planner::GridMap& map)
{
    std::vector<char> occ_2d(map.buffer_size_2d, 0);
    std::vector<char> occ_3d(map.buffer_size_3d, 0);
    map.loadMap(occ_2d, occ_3d);
}

// ################################
// C++: Synthetic MomaTraj: constant yaw, arc = v * t  → XY integrates along +x
// ################################
nmoma_planner::MomaTraj makeStraightTraj(const Eigen::Vector3d& start_xy_yaw,
                                        double duration,
                                        double arc_vel,
                                        size_t dof_num)
{
    nmoma_planner::CoefficientMat<9, 5> c =
        nmoma_planner::CoefficientMat<9, 5>::Zero();
    // col(Order)=const, col(Order-1)=linear  (see Piece::getPos)
    c(0, 5) = start_xy_yaw.z();
    c(1, 4) = arc_vel;
    std::vector<double> durs = {duration};
    std::vector<nmoma_planner::CoefficientMat<9, 5>> mats = {c};
    return nmoma_planner::MomaTraj(
        nmoma_planner::PolyTrajectory<9, 5>(durs, mats), start_xy_yaw, dof_num);
}

// ################################
// C++: Synthetic MomaTraj: fixed XY, yaw = yaw0 + omega * t
// ################################
nmoma_planner::MomaTraj makeYawSweepTraj(const Eigen::Vector3d& start_xy_yaw,
                                        double duration,
                                        double yaw_rate,
                                        size_t dof_num)
{
    nmoma_planner::CoefficientMat<9, 5> c =
        nmoma_planner::CoefficientMat<9, 5>::Zero();
    c(0, 5) = start_xy_yaw.z();
    c(0, 4) = yaw_rate;
    c(1, 5) = 0.0;
    std::vector<double> durs = {duration};
    std::vector<nmoma_planner::CoefficientMat<9, 5>> mats = {c};
    return nmoma_planner::MomaTraj(
        nmoma_planner::PolyTrajectory<9, 5>(durs, mats), start_xy_yaw, dof_num);
}
}  // namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_trajectory_collision_checker",
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
    nh.setParam("grid_map/map_size_x", 8.0);
    nh.setParam("grid_map/map_size_y", 8.0);
    nh.setParam("grid_map/map_size_z", 2.0);
    nh.setParam("grid_map/resolution", 0.10);
    nh.setParam("agent/mode", std::string("planner"));
    nh.setParam("agent/fixed_sequence", false);
    nh.setParam("grid_map/use_rog", false);

    auto grid_map = std::make_shared<nmoma_planner::GridMap>();
    grid_map->init(nh);
    grid_map->setMomaParam(profile);
    const double res = 0.05;
    const size_t dof = profile->dof_num;

    // Case A — empty map, straight traj: expect true (safe)
    {
        loadEmptyMap(*grid_map);
        auto traj = makeStraightTraj(Eigen::Vector3d(-1.5, 0.0, 0.0), 3.0, 1.0, dof);
        const bool ok =
            nmoma_planner::checkWholeBodyTrajectoryCollision(grid_map, traj, res);
        if (!ok)
        {
            std::cerr << "Case A clear traj FAILED (expected true)\n";
            return 1;
        }
        std::cout << "Case A clear traj PASSED\n";
    }

    // Case B — mid-path box penetration; endpoints safe: expect false
    {
        const Eigen::Vector3d obs(0.0, 0.0, 0.30);
        loadObstacleAt(*grid_map, obs, 0.10, profile->chassis_height);

        auto traj = makeStraightTraj(Eigen::Vector3d(-2.0, 0.0, 0.0), 4.0, 1.0, dof);
        const Eigen::VectorXd start = traj.getState(0.0);
        const Eigen::VectorXd mid = traj.getState(2.0);
        const Eigen::VectorXd end = traj.getState(4.0);
        if (grid_map->isWholeBodyCollision(start) || grid_map->isWholeBodyCollision(end))
        {
            std::cerr << "Case B setup FAILED (endpoints must be safe)\n";
            return 1;
        }
        if (!grid_map->isWholeBodyCollision(mid))
        {
            std::cerr << "Case B setup FAILED (mid must penetrate box)\n";
            return 1;
        }
        const bool ok =
            nmoma_planner::checkWholeBodyTrajectoryCollision(grid_map, traj, res);
        if (ok)
        {
            std::cerr << "Case B mid box penetration FAILED (expected false)\n";
            return 1;
        }
        std::cout << "Case B mid box penetration PASSED\n";
    }

    // Case C — yaw sweep: endpoints safe, mid yaw box face hits: expect false
    {
        Eigen::Vector3d side_local = Eigen::Vector3d::Zero();
        double best_abs_y = 0.0;
        for (const CollisionSphere& s : profile->base_obstacle_proxies_)
        {
            // ################################
            // C++: Outermost ±y face sphere (small |x|) for yaw-sweep isolation
            // ################################
            if (std::abs(s.local_offset.x()) > 0.08)
            {
                continue;
            }
            if (std::abs(s.local_offset.y()) > best_abs_y)
            {
                best_abs_y = std::abs(s.local_offset.y());
                side_local = s.local_offset;
            }
        }
        if (best_abs_y < 0.2)
        {
            std::cerr << "Case C setup FAILED (need near-face +y/-y box sphere)\n";
            return 1;
        }

        auto state_at_yaw = [&](double yaw) {
            Eigen::VectorXd st = Eigen::VectorXd::Zero(3 + static_cast<int>(dof));
            st(2) = yaw;
            return st;
        };

        double yaw_hit = std::numeric_limits<double>::quiet_NaN();
        double yaw0 = std::numeric_limits<double>::quiet_NaN();
        double yaw1 = std::numeric_limits<double>::quiet_NaN();
        Eigen::Vector3d obs_center = Eigen::Vector3d::Zero();
        const double face_sign = (side_local.y() >= 0.0) ? 1.0 : -1.0;

        // ################################
        // C++: Try exterior offsets until a colliding mid yaw has safe brackets
        // ################################
        const double extras[] = {0.16, 0.20, 0.24, 0.28, 0.32};
        for (double extra : extras)
        {
            obs_center = Eigen::Vector3d(
                side_local.x(),
                side_local.y() + face_sign * extra,
                side_local.z());
            loadObstacleAt(*grid_map, obs_center, 0.08, profile->chassis_height);

            yaw_hit = std::numeric_limits<double>::quiet_NaN();
            yaw0 = std::numeric_limits<double>::quiet_NaN();
            yaw1 = std::numeric_limits<double>::quiet_NaN();
            for (double y = -0.6; y <= 0.6 + 1e-9; y += 0.05)
            {
                if (grid_map->isWholeBodyCollision(state_at_yaw(y)))
                {
                    yaw_hit = y;
                    break;
                }
            }
            if (!std::isfinite(yaw_hit))
            {
                continue;
            }
            for (double y = yaw_hit - 0.05; y >= -M_PI; y -= 0.05)
            {
                if (!grid_map->isWholeBodyCollision(state_at_yaw(y)))
                {
                    yaw0 = y;
                    break;
                }
            }
            for (double y = yaw_hit + 0.05; y <= M_PI; y += 0.05)
            {
                if (!grid_map->isWholeBodyCollision(state_at_yaw(y)))
                {
                    yaw1 = y;
                    break;
                }
            }
            if (std::isfinite(yaw0) && std::isfinite(yaw1) && yaw0 < yaw_hit && yaw_hit < yaw1)
            {
                break;
            }
        }

        if (!std::isfinite(yaw0) || !std::isfinite(yaw1) || !std::isfinite(yaw_hit))
        {
            std::cerr << "Case C setup FAILED (could not bracket hit yaw with safe ends)"
                      << " yaw0=" << yaw0 << " hit=" << yaw_hit << " yaw1=" << yaw1
                      << " side=" << side_local.transpose()
                      << " obs=" << obs_center.transpose() << "\n";
            return 1;
        }

        const double T = 2.0;
        auto traj = makeYawSweepTraj(
            Eigen::Vector3d(0.0, 0.0, yaw0), T, (yaw1 - yaw0) / T, dof);
        const Eigen::VectorXd start = traj.getState(0.0);
        const Eigen::VectorXd mid = traj.getState(T * (yaw_hit - yaw0) / (yaw1 - yaw0));
        const Eigen::VectorXd end = traj.getState(T);
        if (grid_map->isWholeBodyCollision(start) || grid_map->isWholeBodyCollision(end))
        {
            std::cerr << "Case C setup FAILED (endpoints must be safe)\n";
            return 1;
        }
        if (!grid_map->isWholeBodyCollision(mid))
        {
            std::cerr << "Case C setup FAILED (mid yaw must collide box)\n";
            return 1;
        }
        const bool ok =
            nmoma_planner::checkWholeBodyTrajectoryCollision(grid_map, traj, res);
        if (ok)
        {
            std::cerr << "Case C yaw sweep FAILED (expected false)\n";
            return 1;
        }
        std::cout << "Case C yaw sweep PASSED\n";
    }

    std::cout << "trajectory_collision_checker regression PASSED\n";
    return 0;
}
