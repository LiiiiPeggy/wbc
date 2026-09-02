#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <ros/ros.h>

#include "plan_env/grid_map.h"

namespace {

ros::NodeHandle makeNodeHandle(const std::string& test_name) {
  return ros::NodeHandle("~" + test_name);
}

void configureSmallStaticMap(ros::NodeHandle& nh) {
  nh.setParam("environment_mode", "static_empty");
  nh.setParam("environment/static_empty/size_x", 1.0);
  nh.setParam("environment/static_empty/size_y", 0.8);
  nh.setParam("environment/static_empty/size_z", 0.6);
  nh.setParam("environment/static_empty/resolution", 0.2);
}

TEST(StaticEmptyGridMap, DefaultsDefineCanonicalBoundsAndResolution) {
  ros::NodeHandle nh = makeNodeHandle("static_empty_defaults");
  nh.setParam("environment_mode", "static_empty");

  GridMap map;
  map.initMap(nh);

  Eigen::Vector3d origin;
  Eigen::Vector3d size;
  map.getRegion(origin, size);

  EXPECT_EQ(GridMap::EnvironmentMode::StaticEmpty, map.getEnvironmentMode());
  EXPECT_DOUBLE_EQ(0.05, map.getResolution());
  EXPECT_TRUE(size.isApprox(Eigen::Vector3d(16.0, 12.0, 3.0)));
  EXPECT_TRUE(origin.isApprox(Eigen::Vector3d(-8.0, -6.0, 0.0)));
}

TEST(StaticEmptyGridMap, CanonicalConfigOverridesLegacyGridSize) {
  ros::NodeHandle nh = makeNodeHandle("static_empty_configured");
  nh.setParam("environment_mode", "static_empty");
  nh.setParam("environment/static_empty/size_x", 4.0);
  nh.setParam("environment/static_empty/size_y", 2.0);
  nh.setParam("environment/static_empty/size_z", 1.5);
  nh.setParam("environment/static_empty/resolution", 0.25);
  nh.setParam("grid_map/map_size_x", 99.0);
  nh.setParam("grid_map/map_size_y", 98.0);
  nh.setParam("grid_map/map_size_z", 97.0);
  nh.setParam("grid_map/resolution", 0.9);
  nh.setParam("grid_map/ground_height", -0.5);

  GridMap map;
  map.initMap(nh);

  Eigen::Vector3d origin;
  Eigen::Vector3d size;
  map.getRegion(origin, size);

  EXPECT_DOUBLE_EQ(0.25, map.getResolution());
  EXPECT_TRUE(size.isApprox(Eigen::Vector3d(4.0, 2.0, 1.5)));
  EXPECT_TRUE(origin.isApprox(Eigen::Vector3d(-2.0, -1.0, -0.5)));
  EXPECT_TRUE(map.isInMap(Eigen::Vector3d(1.99, 0.0, 0.0)));
  EXPECT_FALSE(map.isInMap(Eigen::Vector3d(2.01, 0.0, 0.0)));
}

TEST(StaticEmptyGridMap, InitializesEveryVoxelKnownFreeWithFiniteDistance) {
  ros::NodeHandle nh = makeNodeHandle("static_empty_queryable");
  configureSmallStaticMap(nh);

  GridMap map;
  map.initMap(nh);

  Eigen::Vector3i voxel_num;
  map.getVoxelNum(voxel_num);
  for (int x = 0; x < voxel_num.x(); ++x) {
    for (int y = 0; y < voxel_num.y(); ++y) {
      for (int z = 0; z < voxel_num.z(); ++z) {
        const Eigen::Vector3i index(x, y, z);
        Eigen::Vector3d position;
        map.indexToPos(index, position);
        EXPECT_TRUE(map.isKnownFree(index)) << index.transpose();
        EXPECT_TRUE(std::isfinite(map.getDistance(position)))
            << index.transpose();
      }
    }
  }

  EXPECT_FALSE(map.isInMap(Eigen::Vector3d(0.51, 0.0, 0.2)));
  EXPECT_FALSE(map.isInMap(Eigen::Vector3d(-0.51, 0.0, 0.2)));
  EXPECT_FALSE(map.isInMap(Eigen::Vector3i(-1, 0, 0)));
  EXPECT_FALSE(map.isInMap(voxel_num));
  EXPECT_FALSE(map.isKnownFree(Eigen::Vector3i(-1, 0, 0)));
  EXPECT_FALSE(map.isKnownFree(voxel_num));
}

TEST(StaticEmptyGridMap, RemainsReadyWithoutOnlineSensors) {
  ros::NodeHandle nh = makeNodeHandle("static_empty_no_sensors");
  configureSmallStaticMap(nh);

  GridMap map;
  map.initMap(nh);

  EXPECT_TRUE(map.isReady());
  EXPECT_FALSE(map.usesOnlineSensing());
  EXPECT_FALSE(map.getOdomDepthTimeout());

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::WallDuration(15.1).sleep();

  EXPECT_TRUE(map.isReady());
  EXPECT_FALSE(map.getOdomDepthTimeout());
}

TEST(StaticEmptyGridMap, RejectsInvalidConfigurationValues) {
  const std::vector<std::pair<std::string, double>> invalid_values = {
      {"environment/static_empty/size_x", 0.0},
      {"environment/static_empty/size_y", -1.0},
      {"environment/static_empty/size_z",
       std::numeric_limits<double>::infinity()},
      {"environment/static_empty/resolution",
       std::numeric_limits<double>::quiet_NaN()},
  };

  for (size_t i = 0; i < invalid_values.size(); ++i) {
    ros::NodeHandle nh =
        makeNodeHandle("static_empty_invalid_" + std::to_string(i));
    configureSmallStaticMap(nh);
    nh.setParam(invalid_values[i].first, invalid_values[i].second);

    GridMap map;
    EXPECT_THROW(map.initMap(nh), std::invalid_argument)
        << invalid_values[i].first;
  }
}

TEST(StaticEmptyGridMap, RejectsUnknownEnvironmentMode) {
  ros::NodeHandle nh = makeNodeHandle("static_empty_unknown_mode");
  nh.setParam("environment_mode", "empty-ish");

  GridMap map;
  EXPECT_THROW(map.initMap(nh), std::invalid_argument);
}

TEST(StaticEmptyGridMap, DefaultModePreservesLegacyOnlineConfiguration) {
  ros::NodeHandle nh = makeNodeHandle("simulated_default");
  nh.setParam("grid_map/map_size_x", 3.0);
  nh.setParam("grid_map/map_size_y", 2.0);
  nh.setParam("grid_map/map_size_z", 1.0);
  nh.setParam("grid_map/resolution", 0.5);

  GridMap map;
  map.initMap(nh);

  Eigen::Vector3d origin;
  Eigen::Vector3d size;
  map.getRegion(origin, size);

  EXPECT_EQ(GridMap::EnvironmentMode::Simulated, map.getEnvironmentMode());
  EXPECT_TRUE(map.usesOnlineSensing());
  EXPECT_DOUBLE_EQ(0.5, map.getResolution());
  EXPECT_TRUE(size.isApprox(Eigen::Vector3d(3.0, 2.0, 1.0)));
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_static_empty_grid_map",
            ros::init_options::AnonymousName);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
