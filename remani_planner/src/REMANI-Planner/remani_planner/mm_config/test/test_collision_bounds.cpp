#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>

#include "mm_config/mm_config.hpp"

namespace {

using remani_planner::MMConfig;

void configureMm(ros::NodeHandle& nh) {
  nh.setParam("mm/manipulator_type", std::string("cr10"));
  nh.setParam("mm/mobile_base_dof", 2);
  nh.setParam("mm/mobile_base_length", 1.10);
  nh.setParam("mm/mobile_base_width", 0.90);
  nh.setParam("mm/mobile_base_height", 0.40);
  nh.setParam("mm/mobile_base_check_radius", 0.20);
  nh.setParam("mm/mobile_base_wheel_base", 0.56);
  nh.setParam("mm/mobile_base_wheel_radius", 0.125);
  nh.setParam("mm/mobile_base_max_wheel_omega", 4.0);
  nh.setParam("mm/mobile_base_max_wheel_alpha", 8.0);
  nh.setParam("mm/manipulator_dof", 6);
  nh.setParam("mm/manipulator_thickness", 0.06);
  nh.setParam("mm/manipulator_config",
              std::vector<double>{0.1765, 0.607, 0.568, 0.191, 0.125, 0.1084});
  nh.setParam("mm/manipulator_min_pos", std::vector<double>(6, -3.0));
  nh.setParam("mm/manipulator_max_pos", std::vector<double>(6, 3.0));
  nh.setParam("mm/base_mani_fixed_joint_xyz_ypr",
              std::vector<double>{0.2462, 0.0, 0.1, 0.0, 0.0, 0.0});
  nh.setParam("mm/ee_tcp_xyz_rpy", std::vector<double>(6, 0.0));
  nh.setParam("optimization/safe_margin", 0.05);
  nh.setParam("optimization/safe_margin_mani", 0.05);
  nh.setParam("optimization/self_safe_margin", 0.02);
  nh.setParam("optimization/ground_safe_dis", 0.1);
  nh.setParam("grid_map/resolution", 0.05);
}

std::shared_ptr<GridMap> makeMap(ros::NodeHandle& nh, double ground,
                                 double size_x, double size_y, double size_z) {
  nh.setParam("environment_mode", "static_empty");
  nh.setParam("environment/static_empty/resolution", 0.05);
  nh.setParam("environment/static_empty/size_x", size_x);
  nh.setParam("environment/static_empty/size_y", size_y);
  nh.setParam("environment/static_empty/size_z", size_z);
  nh.setParam("grid_map/ground_height", ground);
  std::shared_ptr<GridMap> map(new GridMap());
  map->initMap(nh);
  return map;
}

struct FaceCase {
  const char* name;
  double ground;
  double size_x;
  double size_y;
  double size_z;
  Eigen::Vector3d car;
};

const std::vector<FaceCase> kBaseFaces = {
    {"-x", 0.0, 1.5, 3.0, 3.0, Eigen::Vector3d(-1.0, 0.0, 0.0)},
    {"+x", 0.0, 1.5, 3.0, 3.0, Eigen::Vector3d(1.0, 0.0, 0.0)},
    {"-y", 0.0, 3.0, 1.5, 3.0, Eigen::Vector3d(0.0, -1.0, 0.0)},
    {"+y", 0.0, 3.0, 1.5, 3.0, Eigen::Vector3d(0.0, 1.0, 0.0)},
    {"-z", 0.3, 3.0, 3.0, 3.0, Eigen::Vector3d::Zero()},
    {"+z", 0.0, 3.0, 3.0, 0.15, Eigen::Vector3d::Zero()},
};

const std::vector<FaceCase> kArmFaces = {
    {"-x", -0.2, 1.5, 3.0, 3.0, Eigen::Vector3d::Zero()},
    {"+x", -0.2, 1.5, 3.0, 3.0, Eigen::Vector3d::Zero()},
    {"-y", -0.2, 3.0, 1.5, 3.0, Eigen::Vector3d::Zero()},
    {"+y", -0.2, 3.0, 1.5, 3.0, Eigen::Vector3d::Zero()},
    {"-z", 0.0, 3.0, 3.0, 3.0, Eigen::Vector3d::Zero()},
    {"+z", -0.2, 3.0, 3.0, 0.7, Eigen::Vector3d::Zero()},
};

TEST(CollisionBounds, BaseFootprintRejectsAllSixOutOfMapFaces) {
  for (size_t i = 0; i < kBaseFaces.size(); ++i) {
    ros::NodeHandle nh("~base_face_" + std::to_string(i));
    configureMm(nh);
    const FaceCase& face = kBaseFaces[i];
    const auto map = makeMap(nh, face.ground, face.size_x, face.size_y, face.size_z);
    MMConfig config;
    config.setParam(nh, map);
    double min_dist = -1.0;
    EXPECT_TRUE(config.checkCarObsCollision(face.car, true, true, min_dist))
        << face.name;
    EXPECT_DOUBLE_EQ(0.0, min_dist) << face.name;
  }
}

bool allBaseSamplesInMap(MMConfig& config, GridMap& map) {
  const Eigen::Matrix4d mount = config.getTq0();
  const Eigen::Matrix4Xd samples = config.getBaseLinkPoint();
  for (int i = 0; i < samples.cols(); ++i) {
    const Eigen::Vector3d point = (mount * samples.col(i)).head(3);
    if (!map.isInMap(point)) {
      return false;
    }
  }
  return true;
}

bool sampleCrossesFace(GridMap& map, const Eigen::Vector3d& point,
                       const char* face) {
  Eigen::Vector3d origin, size;
  map.getRegion(origin, size);
  const std::string name(face);
  if (name == "-x") return point.x() < origin.x();
  if (name == "+x") return point.x() > origin.x() + size.x();
  if (name == "-y") return point.y() < origin.y();
  if (name == "+y") return point.y() > origin.y() + size.y();
  if (name == "-z") return point.z() < origin.z();
  return point.z() > origin.z() + size.z();
}

bool armSampleCrossesFace(MMConfig& config, GridMap& map,
                          const Eigen::VectorXd& q, const char* face) {
  Eigen::Matrix4d transform = config.getTq0();
  std::vector<Eigen::Matrix4d> joints, unused;
  config.getJointTrans(q, joints, unused);
  const std::vector<Eigen::Matrix4Xd> links = config.getLinkPoint();
  for (size_t i = 0; i < links.size(); ++i) {
    transform = transform * joints[i];
    for (int j = 0; j < links[i].cols(); ++j) {
      const Eigen::Vector3d point = (transform * links[i].col(j)).head(3);
      if (sampleCrossesFace(map, point, face)) {
        return true;
      }
    }
  }
  return false;
}

TEST(CollisionBounds, ArmLinksRejectAllSixOutOfMapFaces) {
  for (size_t i = 0; i < kArmFaces.size(); ++i) {
    ros::NodeHandle nh("~arm_face_" + std::to_string(i));
    configureMm(nh);
    const FaceCase& face = kArmFaces[i];
    const auto map = makeMap(nh, face.ground, face.size_x, face.size_y, face.size_z);
    // Isolate map-boundary admission from the independent ground-clearance rule.
    nh.setParam("optimization/ground_safe_dis", -1.0);
    MMConfig config;
    config.setParam(nh, map);
    ASSERT_TRUE(allBaseSamplesInMap(config, *map)) << face.name;

    Eigen::VectorXd q(6);
    bool found_crossing = false;
    const std::vector<double> angles = {-3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0};
    for (double q0 : angles) {
      for (double q1 : angles) {
        for (double q2 : angles) {
          q << q0, q1, q2, 0.0, 0.0, 0.0;
          if (armSampleCrossesFace(config, *map, q, face.name)) {
            found_crossing = true;
            break;
          }
        }
        if (found_crossing) break;
      }
      if (found_crossing) break;
    }
    ASSERT_TRUE(found_crossing) << face.name;
    double min_dist = -1.0;
    EXPECT_TRUE(config.checkManiObsCollision(Eigen::Vector3d::Zero(), q, false, min_dist))
        << face.name;
    EXPECT_DOUBLE_EQ(0.0, min_dist) << face.name;
  }
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_collision_bounds", ros::init_options::AnonymousName);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
