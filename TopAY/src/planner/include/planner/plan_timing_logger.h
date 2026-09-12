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
    void logPlan(const std::string& tag,
                 double topo_ms,
                 double mcrrt_ms,
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
