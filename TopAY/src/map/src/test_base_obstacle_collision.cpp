// ################################
// C++: Ranger base-obstacle GridMap runtime collision (no continuous cost)
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

double boxTopFromProxies(const MomaParam& profile)
{
    double zmax = 0.0;
    for (const CollisionSphere& s : profile.base_obstacle_proxies_)
    {
        zmax = std::max(zmax, s.local_offset.z() + s.obstacle_radius);
    }
    return zmax;
}

double boxMidZFromProxies(const MomaParam& profile)
{
    double zsum = 0.0;
    for (const CollisionSphere& s : profile.base_obstacle_proxies_)
    {
        zsum += s.local_offset.z();
    }
    return zsum / static_cast<double>(profile.base_obstacle_proxies_.size());
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
    const Eigen::VectorXd home = Eigen::VectorXd::Zero(9);
    const double mid_z = boxMidZFromProxies(*profile);
    const double top_z = boxTopFromProxies(*profile);

    // Case A — mid-height obstacle intersects box envelope
    loadObstacleAt(grid_map, Eigen::Vector3d(0.0, 0.0, mid_z), 0.08, profile->chassis_height);
    if (!grid_map.isWholeBodyCollision(home))
    {
        std::cerr << "Case A box-mid obstacle FAILED (expected collision=true)\n";
        return 1;
    }
    std::cout << "Case A box-mid obstacle PASSED\n";

    // Case B — obstacle above box top clearance
    loadObstacleAt(
        grid_map, Eigen::Vector3d(0.0, 0.0, top_z + 0.15), 0.08, profile->chassis_height);
    if (grid_map.isWholeBodyCollision(home))
    {
        std::cerr << "Case B above-box FAILED (expected collision=false)\n";
        return 1;
    }
    std::cout << "Case B above-box clearance PASSED\n";

    std::cout << "Base obstacle GridMap collision PASSED\n";
    return 0;
}
