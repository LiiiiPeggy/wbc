#pragma once

#include <memory>

#include <mm_config/mm_config.hpp>
#include <plan_env/grid_map.h>

#include <remani_real/candidate_validator.hpp>

namespace remani_real {

// ################################
// C++: MMConfig-backed validation environment begin
// ################################
class MmConfigValidationEnvironment : public ValidationEnvironment {
 public:
  MmConfigValidationEnvironment(std::shared_ptr<GridMap> map,
                                std::shared_ptr<remani_planner::MMConfig> config);

  bool samplesInMap(const Eigen::Vector3d& car_state,
                    const Eigen::VectorXd& q) const override;
  bool inCollision(const Eigen::Vector3d& car_state, const Eigen::VectorXd& q,
                   int* coll_type) const override;
  Eigen::Matrix4d eePose(const Eigen::Vector3d& car_state,
                         const Eigen::VectorXd& q) const override;

 private:
  std::shared_ptr<GridMap> map_;
  std::shared_ptr<remani_planner::MMConfig> config_;
};
// ################################
// C++: MMConfig-backed validation environment end
// ################################

}  // namespace remani_real
