// ################################
// C++: Dense whole-body trajectory hard validation via GridMap::isWholeBodyCollision
// ################################
#include "planner/trajectory_collision_checker.h"

#include "map/grid_map.h"
#include "planner/moma_traj_opt.h"

#include <algorithm>
#include <cmath>

namespace nmoma_planner
{
bool checkWholeBodyTrajectoryCollision(GridMap& grid_map,
                                       const MomaTraj& traj,
                                       double resolution)
{
    if (!traj.is_init)
    {
        return false;
    }
    const double T = traj.getTotalDuration();
    if (!(T >= 0.0) || !std::isfinite(T))
    {
        return false;
    }
    const double res = std::max(resolution, 1e-4);
    for (double t = 0.0; t < T; t += res)
    {
        if (grid_map.isWholeBodyCollision(traj.getState(t)))
        {
            return false;
        }
    }
    // ################################
    // C++: Always validate the exact endpoint (loop may skip when T % res != 0)
    // ################################
    if (grid_map.isWholeBodyCollision(traj.getState(T)))
    {
        return false;
    }
    return true;
}

bool checkWholeBodyTrajectoryCollision(const std::shared_ptr<GridMap>& grid_map,
                                       const MomaTraj& traj,
                                       double resolution)
{
    if (!grid_map)
    {
        return false;
    }
    return checkWholeBodyTrajectoryCollision(*grid_map, traj, resolution);
}
}  // namespace nmoma_planner
