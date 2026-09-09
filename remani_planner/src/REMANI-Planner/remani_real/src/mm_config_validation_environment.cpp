#include <remani_real/mm_config_validation_environment.hpp>

#include <cmath>
#include <vector>

namespace remani_real {

// ################################
// C++: MMConfig validation environment begin
// ################################
MmConfigValidationEnvironment::MmConfigValidationEnvironment(
    std::shared_ptr<GridMap> map,
    std::shared_ptr<remani_planner::MMConfig> config)
    : map_(std::move(map)), config_(std::move(config)) {}

bool MmConfigValidationEnvironment::samplesInMap(
    const Eigen::Vector3d& car_state, const Eigen::VectorXd& q) const {
  if (!map_ || !config_) {
    return false;
  }
  remani_planner::MMConfig& cfg = *config_;
  // ################################
  // C++: include mobile-base footprint samples in map bounds begin
  // ################################
  std::vector<Eigen::Vector3d> car_pts;
  cfg.getCarPts(car_state, car_pts);
  for (const Eigen::Vector3d& point : car_pts) {
    if (!map_->isInMap(point)) {
      return false;
    }
  }
  // ################################
  // C++: include mobile-base footprint samples in map bounds end
  // ################################

  Eigen::Matrix4d T_car = Eigen::Matrix4d::Identity();
  T_car(0, 3) = car_state(0);
  T_car(1, 3) = car_state(1);
  const double c = std::cos(car_state(2));
  const double s = std::sin(car_state(2));
  T_car(0, 0) = c;
  T_car(0, 1) = -s;
  T_car(1, 0) = s;
  T_car(1, 1) = c;

  const Eigen::Matrix4d mount = T_car * cfg.getTq0();
  const Eigen::Matrix4Xd base_pts = cfg.getBaseLinkPoint();
  for (int i = 0; i < base_pts.cols(); ++i) {
    const Eigen::Vector3d point = (mount * base_pts.col(i)).head(3);
    if (!map_->isInMap(point)) {
      return false;
    }
  }

  std::vector<Eigen::Matrix4d> T_joint;
  std::vector<Eigen::Matrix4d> T_grad;
  cfg.getJointTrans(q, T_joint, T_grad);
  Eigen::Matrix4d T = mount;
  const std::vector<Eigen::Matrix4Xd> link_pts = cfg.getLinkPoint();
  for (size_t j = 0; j < T_joint.size() && j < link_pts.size(); ++j) {
    T = T * T_joint[j];
    for (int i = 0; i < link_pts[j].cols(); ++i) {
      const Eigen::Vector3d point = (T * link_pts[j].col(i)).head(3);
      if (!map_->isInMap(point)) {
        return false;
      }
    }
  }
  return true;
}

bool MmConfigValidationEnvironment::inCollision(
    const Eigen::Vector3d& car_state, const Eigen::VectorXd& q,
    int* coll_type) const {
  if (!config_) {
    return true;
  }
  int type = -1;
  const bool hit = config_->checkcollision(car_state, q, true, type);
  if (coll_type != nullptr) {
    *coll_type = type;
  }
  return hit;
}

Eigen::Matrix4d MmConfigValidationEnvironment::eePose(
    const Eigen::Vector3d& car_state, const Eigen::VectorXd& q) const {
  if (!config_) {
    return Eigen::Matrix4d::Identity();
  }
  return config_->getEePose(car_state, q);
}
// ################################
// C++: MMConfig validation environment end
// ################################

}  // namespace remani_real
