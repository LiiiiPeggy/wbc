// ################################
// C++: Hard-gate short-circuit helper regression (opt fail → no hard run)
// ################################
#include "planner/trajectory_collision_checker.h"

#include <iostream>

int main()
{
    using nmoma_planner::shouldRunWholeBodyTrajHardGate;

    if (shouldRunWholeBodyTrajHardGate(false, true))
    {
        std::cerr << "opt_ok=false must skip hard gate\n";
        return 1;
    }
    if (shouldRunWholeBodyTrajHardGate(true, false))
    {
        std::cerr << "traj.is_init=false must skip hard gate\n";
        return 1;
    }
    if (shouldRunWholeBodyTrajHardGate(false, false))
    {
        std::cerr << "both false must skip hard gate\n";
        return 1;
    }
    if (!shouldRunWholeBodyTrajHardGate(true, true))
    {
        std::cerr << "opt_ok && is_init must run hard gate\n";
        return 1;
    }
    std::cout << "hard_gate_short_circuit PASSED\n";
    return 0;
}
