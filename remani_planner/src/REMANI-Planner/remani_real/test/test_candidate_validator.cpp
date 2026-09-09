#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <mm_config/mm_config.hpp>
#include <plan_env/grid_map.h>

#include <remani_real/candidate_validator.hpp>
#include <remani_real/mm_config_validation_environment.hpp>
#include <remani_real/preview_publisher.hpp>

namespace remani_real {
namespace {

// ################################
// C++: CandidateValidator test fixtures begin
// ################################
CandidateSegment makeSegment(uint32_t id, int singul, double start_time,
                             double duration,
                             const MMController::Piece::CoefficientMat& coeff) {
  CandidateSegment segment;
  segment.trajectory_id = id;
  segment.singul = singul;
  segment.trajectory.emplace_back(duration, coeff);
  segment.start_time = start_time;
  segment.duration = duration;
  return segment;
}

MMController::Piece::CoefficientMat poseCoeff(double x, double y,
                                              const Eigen::Matrix<double, 6, 1>& q,
                                              double vx = 0.0, double vy = 0.0,
                                              const Eigen::Matrix<double, 6, 1>& qd =
                                                  Eigen::Matrix<double, 6, 1>::Zero()) {
  MMController::Piece::CoefficientMat coeff =
      MMController::Piece::CoefficientMat::Zero(8, 8);
  coeff.col(7) << x, y, q(0), q(1), q(2), q(3), q(4), q(5);
  coeff(0, 6) = vx;
  coeff(1, 6) = vy;
  for (int i = 0; i < 6; ++i) {
    coeff(2 + i, 6) = qd(i);
  }
  return coeff;
}

FrozenCandidate makeCandidate(uint64_t id, double start_yaw,
                              const std::vector<CandidateSegment>& segments) {
  return std::make_shared<const CandidateTrajectory>(id, segments, start_yaw,
                                                     ros::Time(1.0));
}

ActualStateSnapshot matchingActual(const FrozenCandidate& candidate) {
  const WholeBodySample start = candidate->sample(0.0);
  ActualStateSnapshot actual;
  actual.base_xy = start.position.head<2>();
  actual.base_yaw = start.base_yaw;
  actual.q = start.position.segment<6>(2);
  actual.odom_valid = true;
  actual.joints_valid = true;
  actual.velocity_valid = true;
  return actual;
}

class FakeEnv : public ValidationEnvironment {
 public:
  bool force_collision{false};
  bool force_out_of_map{false};
  int collision_type{2};

  bool samplesInMap(const Eigen::Vector3d& /*car_state*/,
                    const Eigen::VectorXd& /*q*/) const override {
    return !force_out_of_map;
  }

  bool inCollision(const Eigen::Vector3d& /*car_state*/,
                   const Eigen::VectorXd& /*q*/,
                   int* coll_type) const override {
    if (coll_type != nullptr) {
      *coll_type = collision_type;
    }
    return force_collision;
  }

  Eigen::Matrix4d eePose(const Eigen::Vector3d& car_state,
                         const Eigen::VectorXd& /*q*/) const override {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(0, 3) = car_state(0);
    T(1, 3) = car_state(1);
    T(2, 3) = 0.5;
    return T;
  }
};

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

std::shared_ptr<GridMap> makeMap(ros::NodeHandle& nh, double size_xy) {
  nh.setParam("environment_mode", "static_empty");
  nh.setParam("environment/static_empty/resolution", 0.05);
  nh.setParam("environment/static_empty/size_x", size_xy);
  nh.setParam("environment/static_empty/size_y", size_xy);
  nh.setParam("environment/static_empty/size_z", 3.0);
  nh.setParam("grid_map/ground_height", 0.0);
  std::shared_ptr<GridMap> map(new GridMap());
  map->initMap(nh);
  return map;
}

std::shared_ptr<MmConfigValidationEnvironment> makeMmEnv(
    ros::NodeHandle& nh, double size_xy) {
  auto map = makeMap(nh, size_xy);
  auto cfg = std::make_shared<remani_planner::MMConfig>();
  cfg->setParam(nh, map);
  return std::make_shared<MmConfigValidationEnvironment>(map, cfg);
}
// ################################
// C++: CandidateValidator test fixtures end
// ################################

TEST(CandidateValidator, AcceptsFeasibleCandidate) {
  FakeEnv env;
  CandidateValidator validator(ValidationLimits{}, &env);
  const Eigen::Matrix<double, 6, 1> q = Eigen::Matrix<double, 6, 1>::Zero();
  const auto candidate = makeCandidate(
      1, 0.0,
      {makeSegment(1, 1, 0.0, 1.0, poseCoeff(0.0, 0.0, q, 0.05, 0.0))});
  const ValidationReport report = validator.validate(candidate, matchingActual(candidate));
  EXPECT_TRUE(report.valid);
  EXPECT_TRUE(report.error_code.empty());
  EXPECT_NEAR(1.0, report.duration, 1e-12);
  EXPECT_LE(report.max_base_linear_speed, 0.10 + 1e-9);
}

TEST(CandidateValidator, RejectsStartStateMismatch) {
  FakeEnv env;
  CandidateValidator validator(ValidationLimits{}, &env);
  const Eigen::Matrix<double, 6, 1> q = Eigen::Matrix<double, 6, 1>::Zero();
  const auto candidate = makeCandidate(
      2, 0.0, {makeSegment(1, 1, 0.0, 1.0, poseCoeff(0.0, 0.0, q))});
  ActualStateSnapshot actual = matchingActual(candidate);
  actual.base_xy.x() += 0.2;
  const ValidationReport report = validator.validate(candidate, actual);
  EXPECT_FALSE(report.valid);
  EXPECT_EQ("START_STATE_MISMATCH", report.error_code);
}

TEST(CandidateValidator, RejectsBaseSpeedLimit) {
  FakeEnv env;
  CandidateValidator validator(ValidationLimits{}, &env);
  const Eigen::Matrix<double, 6, 1> q = Eigen::Matrix<double, 6, 1>::Zero();
  const auto candidate = makeCandidate(
      3, 0.0, {makeSegment(1, 1, 0.0, 1.0, poseCoeff(0.0, 0.0, q, 0.2, 0.0))});
  const ValidationReport report =
      validator.validate(candidate, matchingActual(candidate));
  EXPECT_FALSE(report.valid);
  EXPECT_EQ("BASE_SPEED_LIMIT", report.error_code);
}

TEST(CandidateValidator, RejectsBaseAngularSpeedLimit) {
  FakeEnv env;
  CandidateValidator validator(ValidationLimits{}, &env);
  const Eigen::Matrix<double, 6, 1> q = Eigen::Matrix<double, 6, 1>::Zero();
  MMController::Piece::CoefficientMat coeff = poseCoeff(0.0, 0.0, q, 0.05, 0.0);
  // ay(0)=2*coeff(1,5); omega=(vx*ay)/vx^2 = ay/vx → 0.02/0.05=0.4 > 0.15.
  coeff(1, 5) = 0.01;
  const auto candidate =
      makeCandidate(10, 0.0, {makeSegment(1, 1, 0.0, 1.0, coeff)});
  const ValidationReport report =
      validator.validate(candidate, matchingActual(candidate));
  EXPECT_FALSE(report.valid);
  EXPECT_EQ("BASE_SPEED_LIMIT", report.error_code);
  EXPECT_NEAR(1.0, report.duration, 1e-12);
}

TEST(CandidateValidator, RejectsJointSpeedLimit) {
  FakeEnv env;
  CandidateValidator validator(ValidationLimits{}, &env);
  Eigen::Matrix<double, 6, 1> q = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> qd = Eigen::Matrix<double, 6, 1>::Zero();
  qd(2) = 0.25;
  const auto candidate = makeCandidate(
      4, 0.0, {makeSegment(1, 1, 0.0, 1.0, poseCoeff(0.0, 0.0, q, 0.0, 0.0, qd))});
  const ValidationReport report =
      validator.validate(candidate, matchingActual(candidate));
  EXPECT_FALSE(report.valid);
  EXPECT_EQ("JOINT_SPEED_LIMIT", report.error_code);
}

TEST(CandidateValidator, RejectsSegmentContinuityBreak) {
  FakeEnv env;
  CandidateValidator validator(ValidationLimits{}, &env);
  const Eigen::Matrix<double, 6, 1> q = Eigen::Matrix<double, 6, 1>::Zero();
  CandidateSegment first = makeSegment(1, 1, 0.0, 1.0, poseCoeff(0.0, 0.0, q));
  CandidateSegment second = makeSegment(2, 1, 1.0, 1.0, poseCoeff(0.5, 0.0, q));
  const auto candidate = makeCandidate(5, 0.0, {first, second});
  const ValidationReport report =
      validator.validate(candidate, matchingActual(candidate));
  EXPECT_FALSE(report.valid);
  EXPECT_EQ("SEGMENT_CONTINUITY", report.error_code);
}

TEST(CandidateValidator, RejectsMapBoundary) {
  FakeEnv env;
  env.force_out_of_map = true;
  CandidateValidator validator(ValidationLimits{}, &env);
  const Eigen::Matrix<double, 6, 1> q = Eigen::Matrix<double, 6, 1>::Zero();
  const auto candidate = makeCandidate(
      6, 0.0, {makeSegment(1, 1, 0.0, 1.0, poseCoeff(0.0, 0.0, q))});
  const ValidationReport report =
      validator.validate(candidate, matchingActual(candidate));
  EXPECT_FALSE(report.valid);
  EXPECT_EQ("MAP_BOUNDARY", report.error_code);
}

TEST(CandidateValidator, RejectsWholeBodyCollision) {
  FakeEnv env;
  env.force_collision = true;
  env.collision_type = 2;
  CandidateValidator validator(ValidationLimits{}, &env);
  const Eigen::Matrix<double, 6, 1> q = Eigen::Matrix<double, 6, 1>::Zero();
  const auto candidate = makeCandidate(
      7, 0.0, {makeSegment(1, 1, 0.0, 1.0, poseCoeff(0.0, 0.0, q))});
  const ValidationReport report =
      validator.validate(candidate, matchingActual(candidate));
  EXPECT_FALSE(report.valid);
  EXPECT_EQ("WHOLE_BODY_COLLISION", report.error_code);
  EXPECT_NE(std::string::npos, report.detail.find("car-mani"));
}

TEST(CandidateValidator, RealEnvRejectsFoldedArmCollision) {
  ros::NodeHandle nh("~folded");
  configureMm(nh);
  auto env = makeMmEnv(nh, 20.0);
  CandidateValidator validator(ValidationLimits{}, env.get());

  // Prefer a hard self-hit that remains inside the static-empty map.
  Eigen::Matrix<double, 6, 1> q = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Vector3d car = Eigen::Vector3d::Zero();
  int coll_type = -1;
  bool found_collision = false;
  for (double j1 = -2.6; j1 <= 2.6 && !found_collision; j1 += 0.4) {
    for (double j2 = -2.6; j2 <= 2.6 && !found_collision; j2 += 0.4) {
      for (double j4 = -1.8; j4 <= 1.8 && !found_collision; j4 += 0.6) {
        q << 0.0, j1, j2, 0.0, j4, 0.0;
        if (!env->samplesInMap(car, q)) {
          continue;
        }
        if (env->inCollision(car, q, &coll_type) &&
            (coll_type == 2 || coll_type == 3)) {
          found_collision = true;
        }
      }
    }
  }
  ASSERT_TRUE(found_collision) << "no in-map hard self-collision fixture found";

  const auto candidate = makeCandidate(
      8, 0.0, {makeSegment(1, 1, 0.0, 0.5, poseCoeff(0.0, 0.0, q))});
  const ValidationReport report =
      validator.validate(candidate, matchingActual(candidate));
  EXPECT_FALSE(report.valid);
  EXPECT_EQ("WHOLE_BODY_COLLISION", report.error_code);
}

TEST(CandidateValidator, RealEnvRejectsTinyMapBoundary) {
  ros::NodeHandle nh("~tiny_map");
  configureMm(nh);
  auto env = makeMmEnv(nh, 0.4);
  CandidateValidator validator(ValidationLimits{}, env.get());

  const Eigen::Matrix<double, 6, 1> q = Eigen::Matrix<double, 6, 1>::Zero();
  const auto candidate = makeCandidate(
      9, 0.0, {makeSegment(1, 1, 0.0, 0.5, poseCoeff(0.0, 0.0, q))});
  const ValidationReport report =
      validator.validate(candidate, matchingActual(candidate));
  EXPECT_FALSE(report.valid);
  EXPECT_EQ("MAP_BOUNDARY", report.error_code);
}

TEST(CandidateValidator, RealEnvRejectsCarFootprintMapBoundary) {
  ros::NodeHandle nh("~car_footprint");
  configureMm(nh);
  // Prefer a pose where checkcollision already reports car-obs out-of-map, and
  // samplesInMap must agree after including getCarPts.
  auto env = makeMmEnv(nh, 3.0);
  CandidateValidator validator(ValidationLimits{}, env.get());

  const Eigen::Matrix<double, 6, 1> q = Eigen::Matrix<double, 6, 1>::Zero();
  double chosen_x = 0.0;
  bool found = false;
  for (double x = 0.0; x <= 1.40; x += 0.05) {
    Eigen::Vector3d car(x, 0.0, 0.0);
    int coll_type = -1;
    if (env->inCollision(car, q, &coll_type) && coll_type == 0 &&
        !env->samplesInMap(car, q)) {
      chosen_x = x;
      found = true;
      break;
    }
  }
  ASSERT_TRUE(found) << "no car-footprint MAP_BOUNDARY fixture found";

  const auto candidate = makeCandidate(
      11, 0.0, {makeSegment(1, 1, 0.0, 0.5, poseCoeff(chosen_x, 0.0, q))});
  const ValidationReport report =
      validator.validate(candidate, matchingActual(candidate));
  EXPECT_FALSE(report.valid);
  EXPECT_EQ("MAP_BOUNDARY", report.error_code);
  EXPECT_NEAR(0.5, report.duration, 1e-12);
}

TEST(PreviewPublisher, AdvertisesOnlyIsolatedPreviewTopics) {
  ros::NodeHandle nh("~preview");
  PreviewPublisher publisher(nh);
  const std::vector<std::string>& topics = publisher.advertisedTopics();
  ASSERT_EQ(3u, topics.size());
  EXPECT_EQ("/remani/candidate_robot", topics[0]);
  EXPECT_EQ("/remani/candidate_base_path", topics[1]);
  EXPECT_EQ("/remani/candidate_ee_path", topics[2]);
  for (const std::string& topic : topics) {
    EXPECT_EQ(std::string::npos, topic.find("joint_states"));
    EXPECT_NE("/tf", topic);
    EXPECT_NE("/tf_static", topic);
    EXPECT_NE("/odom", topic);
  }
}

}  // namespace
}  // namespace remani_real

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_candidate_validator",
            ros::init_options::AnonymousName);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
