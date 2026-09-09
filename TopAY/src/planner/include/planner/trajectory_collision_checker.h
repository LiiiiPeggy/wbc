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
}  // namespace nmoma_planner
