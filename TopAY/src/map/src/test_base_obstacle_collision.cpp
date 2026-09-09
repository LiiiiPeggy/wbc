// ################################
// C++: Ranger base-obstacle GridMap Cases A–D from production Box STL AABB
// ################################
#include "map/grid_map.h"

#include <ros/package.h>
#include <ros/ros.h>

#include <algorithm>
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
}  // namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_base_obstacle_collision", ros::init_options::AnonymousName);

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
        std::cerr << "production YAML must enable box_obstacle with spheres\n";
        return 1;
    }

    ros::NodeHandle nh("~");
    nh.setParam("grid_map/map_size_x", 4.0);
    nh.setParam("grid_map/map_size_y", 4.0);
    nh.setParam("grid_map/map_size_z", 2.0);
    nh.setParam("grid_map/resolution", 0.10);
    nh.setParam("agent/mode", std::string("planner"));
    nh.setParam("agent/fixed_sequence", false);
    nh.setParam("grid_map/use_rog", false);

    nmoma_planner::GridMap grid_map;
    grid_map.init(nh);
    grid_map.setMomaParam(profile);
    const Eigen::VectorXd home = Eigen::VectorXd::Zero(3 + profile->dof_num);

    // ################################
    // C++: Physical box_link.STL AABB in planning frame (audit_ranger_geometry.py)
    // ################################
    const Eigen::Vector3d stl_vmin(-0.60627484, -0.40000001, 0.073);
    const Eigen::Vector3d stl_vmax(0.60000002, 0.41000000, 0.47210598);
    const Eigen::Vector3d stl_center = 0.5 * (stl_vmin + stl_vmax);
    const double margin = 0.02;
    const double obs_r = 0.05;

    // Case A — clear of physical STL + margin (must be free)
    {
        const Eigen::Vector3d center(
            stl_vmax.x() + margin + obs_r + 0.15,
            stl_center.y(),
            stl_center.z());
        loadObstacleAt(grid_map, center, obs_r, profile->chassis_height);
        if (grid_map.isWholeBodyCollision(home))
        {
            std::cerr << "Case A safe-outside FAILED (expected collision=false)\n";
            return 1;
        }
        std::cout << "Case A safe-outside PASSED\n";
    }

    // Case B — obstacle overlaps a production box proxy sphere (must collide)
    {
        const Eigen::Vector3d center(0.0, 0.0, 0.30);
        loadObstacleAt(grid_map, center, 0.10, profile->chassis_height);
        if (!grid_map.isWholeBodyCollision(home))
        {
            std::cerr << "Case B STL-overlap FAILED (expected collision=true)\n";
            return 1;
        }
        std::cout << "Case B STL-overlap collision PASSED\n";
    }

    // Case C — above physical STL top + margin
    {
        const Eigen::Vector3d center(
            stl_center.x(),
            stl_center.y(),
            stl_vmax.z() + margin + obs_r + 0.12);
        loadObstacleAt(grid_map, center, obs_r, profile->chassis_height);
        if (grid_map.isWholeBodyCollision(home))
        {
            std::cerr << "Case C above-box FAILED (expected collision=false)\n";
            return 1;
        }
        std::cout << "Case C above-box clearance PASSED\n";
    }

    // Case D — near STL side but outside STL+margin (false-positive gate)
    {
        const Eigen::Vector3d center(
            stl_vmax.x() + margin + obs_r + 0.04,
            stl_center.y(),
            stl_center.z());
        loadObstacleAt(grid_map, center, obs_r, profile->chassis_height);
        if (grid_map.isWholeBodyCollision(home))
        {
            std::cerr << "Case D near-but-safe FAILED (expected collision=false; "
                         "proxy over-conservative vs physical STL)\n";
            return 1;
        }
        std::cout << "Case D near-but-not-touching PASSED\n";
    }

    std::cout << "Base obstacle GridMap collision PASSED\n";
    return 0;
}
