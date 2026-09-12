// ################################
// C++: Bridge opening free for chassis 2D; home arm hits 1.5 m lintel; pillar collides
// ################################
#include "map/grid_map.h"
#include "random_map_generator/random_map.hpp"

#include <ros/package.h>
#include <ros/ros.h>

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

void stampCloud(nmoma_planner::GridMap& map,
                const pcl::PointCloud<pcl::PointXYZ>& cloud,
                double chassis_height)
{
    std::vector<char> occ_2d(map.buffer_size_2d, 0);
    std::vector<char> occ_3d(map.buffer_size_3d, 0);
    for (const auto& pt : cloud.points)
    {
        Eigen::Vector3d pos(pt.x, pt.y, pt.z);
        Eigen::Vector3i id3;
        map.posToIndex3d(pos, id3);
        if (map.isInMap3d(id3))
        {
            occ_3d[map.toAddress3d(id3)] = 1;
        }
        Eigen::Vector2i id2;
        map.posToIndex2d(pos.head<2>(), id2);
        if (map.isInMap2d(id2) && pt.z < chassis_height)
        {
            occ_2d[map.toAddress2d(id2.x(), id2.y())] = 1;
        }
    }
    map.loadMap(occ_2d, occ_3d);
}
}  // namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_bridge_obstacle_clearance",
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

    ros::NodeHandle nh("~");
    nh.setParam("grid_map/map_size_x", 8.0);
    nh.setParam("grid_map/map_size_y", 8.0);
    // ################################
    // C++: Map Z must cover lintel above 1.5 m clearance
    // ################################
    nh.setParam("grid_map/map_size_z", 2.5);
    nh.setParam("grid_map/resolution", 0.05);
    nh.setParam("agent/mode", std::string("planner"));
    nh.setParam("agent/fixed_sequence", false);
    nh.setParam("grid_map/use_rog", false);

    nmoma_planner::GridMap grid_map;
    grid_map.init(nh);
    grid_map.setMomaParam(profile);

    nmoma_planner::random_map::RandomPCGenerator gen;
    gen.resolution = 0.05;
    gen.size_x = 8.0;
    gen.size_y = 8.0;

    const double opening_width = 3.0;
    const double opening_depth = 0.30;
    // ################################
    // C++: Opening clearance (lintel bottom) raised to 1.5 m
    // ################################
    const double clearance = 1.5;
    const double lintel_th = 0.10;
    const double pillar_w = 0.20;
    const Eigen::Vector3d center(0.0, 0.0, 0.0);

    pcl::PointCloud<pcl::PointXYZ> cloud;
    std::vector<nmoma_planner::random_map::Box::array_repr> reps;
    std::tie(cloud, reps) = gen.generateBridge(
        center, opening_width, opening_depth, clearance, lintel_th, pillar_w, 0.0);
    if (cloud.empty() || reps.size() < 3)
    {
        std::cerr << "generateBridge produced empty geometry\n";
        return 1;
    }
    stampCloud(grid_map, cloud, profile->chassis_height);

    Eigen::VectorXd home = Eigen::VectorXd::Zero(3 + profile->dof_num);

    // Case A — chassis 2D free under opening
    {
        if (grid_map.isCollision2d(home.head<2>(), profile->chassis_colli_radius))
        {
            std::cerr << "Case A chassis-opening 2D free FAILED\n";
            return 1;
        }
        std::cout << "Case A chassis-opening 2D free PASSED\n";
    }

    // ################################
    // C++: Case B — CR10 home arm reaches z≈1.57 m and hits 1.5 m lintel
    // ################################
    {
        int coll_type = -1;
        if (!grid_map.isWholeBodyCollision(home, coll_type))
        {
            std::cerr << "Case B arm-vs-lintel whole-body collision FAILED\n";
            return 1;
        }
        if (coll_type != 1)
        {
            std::cerr << "Case B expected env hit (coll_type=1) got " << coll_type << "\n";
            return 1;
        }
        std::cout << "Case B arm-vs-lintel whole-body collision PASSED\n";
    }

    // Case C — pillar footprint collides
    {
        Eigen::VectorXd at_pillar = home;
        // Left pillar center ≈ (-opening_width/2 - pillar_w/2, 0) = (-1.1, 0)
        at_pillar(0) = -(0.5 * opening_width + 0.5 * pillar_w);
        at_pillar(1) = 0.0;
        if (!grid_map.isWholeBodyCollision(at_pillar))
        {
            std::cerr << "Case C pillar collision FAILED\n";
            return 1;
        }
        std::cout << "Case C pillar collision PASSED\n";
    }

    std::cout << "bridge_obstacle_clearance PASSED\n";
    return 0;
}
