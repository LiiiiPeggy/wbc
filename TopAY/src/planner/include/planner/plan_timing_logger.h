#pragma once

#include <string>

namespace nmoma_planner
{
// ################################
// C++: Wall-clock plan stage timings → TopAY/src/logs/plan_*.log
// ################################
class PlanTimingLogger
{
public:
    void startSession(const std::string& robot_name = "unknown");
    // ################################
    // C++: front_ms = MCRRT or OMPL wall time; total_ms = whole-plan wall latency
    // ################################
    void logPlan(const std::string& tag,
                 const std::string& front,
                 double topo_ms,
                 double front_ms,
                 double opt_ms,
                 double hard_ms,
                 double total_ms,
                 bool success);

private:
    std::string log_path_;
    bool enabled_ = false;
    bool open_failed_logged_ = false;
};
}  // namespace nmoma_planner
