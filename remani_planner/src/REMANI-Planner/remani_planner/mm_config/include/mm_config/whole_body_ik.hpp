// ################################
// C++: whole-body IK solver API begin
// ################################
#ifndef _WHOLE_BODY_IK_HPP_
#define _WHOLE_BODY_IK_HPP_

#include <mm_config/whole_body_ik_types.hpp>
#include <mm_config/fixed_base_arm_ik.hpp>
#include <memory>
#include "plan_env/grid_map.h"

namespace remani_planner
{

// ################################
// C++: ranking helpers (Task 8) begin
// ################################
double computeJointLimitMargin(MMConfig &cfg, const Eigen::VectorXd &q);
double computeCandidateCost(const WholeBodyIkParams &p,
                            const WholeBodyGoalCandidate &c);
bool passesFastPathQuality(const WholeBodyIkParams &p,
                           const WholeBodyGoalCandidate &c);
// ################################
// C++: ranking helpers (Task 8) end
// ################################

class WholeBodyIkSolver {
public:
    using Ptr = std::shared_ptr<WholeBodyIkSolver>;

    WholeBodyIkSolver(MMConfig::Ptr cfg, GridMap::Ptr map, WholeBodyIkParams params);

    WholeBodyIkResult solve(const Eigen::Matrix<double, 9, 1> &xi_start,
                            const Eigen::Matrix4d &T_goal);

    // ################################
    // C++: Stage B public for Milestone C tests begin
    // ################################
    bool runStageB(const Eigen::Matrix<double, 9, 1> &xi0,
                   const Eigen::Matrix4d &T_goal,
                   WholeBodyGoalCandidate &out_cand);
    // ################################
    // C++: Stage B public for Milestone C tests end
    // ################################

private:
    MMConfig::Ptr cfg_;
    GridMap::Ptr map_;
    WholeBodyIkParams params_;
    FixedBaseArmIk arm_ik_;

    FixedBaseArmIkParams makeArmIkParams() const;

    bool runStageC(const Eigen::Matrix<double, 9, 1> &xi0,
                   const Eigen::Matrix4d &T_goal,
                   std::vector<WholeBodyGoalCandidate> &out_cands);
};

} // namespace remani_planner

#endif
// ################################
// C++: whole-body IK solver API end
// ################################
