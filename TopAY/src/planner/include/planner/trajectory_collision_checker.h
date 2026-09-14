#pragma once

// ################################
// C++: Unified whole-body trajectory collision gate (reuses GridMap::isWholeBodyCollision)
// ################################

#include <memory>

namespace nmoma_planner
{
class GridMap;
struct MomaTraj;

// ################################
// C++: Returns true iff every dense sample is collision-free (safe to publish)
// ################################
bool checkWholeBodyTrajectoryCollision(const std::shared_ptr<GridMap>& grid_map,
                                       const MomaTraj& traj,
                                       double resolution);

bool checkWholeBodyTrajectoryCollision(GridMap& grid_map,
                                       const MomaTraj& traj,
                                       double resolution);

// ################################
// C++: Hard gate only after opt+print success and initialized traj
// ################################
inline bool shouldRunWholeBodyTrajHardGate(bool opt_ok, bool traj_is_init)
{
    return opt_ok && traj_is_init;
}
}  // namespace nmoma_planner
