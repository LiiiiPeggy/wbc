// ################################
// C++: Append wall-clock plan stage lines under TopAY/src/logs/
// ################################
#include "planner/plan_timing_logger.h"

#include <ros/package.h>
#include <ros/ros.h>

#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>

namespace nmoma_planner
{
namespace
{
std::string resolveLogsDir()
{
    const std::string pkg = ros::package::getPath("planner");
    if (pkg.empty())
    {
        return {};
    }
    // Prefer .../TopAY/src/logs when package is .../TopAY/src/planner
    const std::string sibling = pkg + "/../logs";
    return sibling;
}

bool ensureDir(const std::string& path)
{
    if (path.empty())
    {
        return false;
    }
    struct stat st;
    if (stat(path.c_str(), &st) == 0)
    {
        return S_ISDIR(st.st_mode);
    }
    return mkdir(path.c_str(), 0755) == 0;
}

std::string isoNow()
{
    using clock = std::chrono::system_clock;
    const auto now = clock::now();
    const std::time_t t = clock::to_time_t(now);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        now.time_since_epoch())
                        .count() %
                    1000;
    std::tm tm_buf;
    gmtime_r(&t, &tm_buf);
    std::ostringstream oss;
    oss << std::put_time(&tm_buf, "%Y-%m-%dT%H:%M:%S")
        << '.' << std::setw(3) << std::setfill('0') << ms << 'Z';
    return oss.str();
}

std::string sessionFileName()
{
    using clock = std::chrono::system_clock;
    const auto now = clock::now();
    const std::time_t t = clock::to_time_t(now);
    std::tm tm_buf;
    localtime_r(&t, &tm_buf);
    std::ostringstream oss;
    oss << "plan_" << std::put_time(&tm_buf, "%Y%m%d_%H%M%S") << ".log";
    return oss.str();
}
}  // namespace

void PlanTimingLogger::startSession(const std::string& robot_name)
{
    const std::string dir = resolveLogsDir();
    if (!ensureDir(dir))
    {
        if (!open_failed_logged_)
        {
            ROS_ERROR("[PlanTiming] cannot create logs dir: %s", dir.c_str());
            open_failed_logged_ = true;
        }
        enabled_ = false;
        return;
    }
    log_path_ = dir + "/" + sessionFileName();
    std::ofstream ofs(log_path_, std::ios::out | std::ios::app);
    if (!ofs)
    {
        if (!open_failed_logged_)
        {
            ROS_ERROR("[PlanTiming] cannot open log: %s", log_path_.c_str());
            open_failed_logged_ = true;
        }
        enabled_ = false;
        return;
    }
    ofs << "# session_start=" << isoNow() << " robot=" << robot_name << "\n";
    enabled_ = true;
    ROS_INFO("[PlanTiming] writing to %s", log_path_.c_str());
}

void PlanTimingLogger::logPlan(const std::string& tag,
                               double topo_ms,
                               double mcrrt_ms,
                               double opt_ms,
                               double hard_ms,
                               double total_ms,
                               bool success)
{
    if (!enabled_)
    {
        return;
    }
    std::ofstream ofs(log_path_, std::ios::out | std::ios::app);
    if (!ofs)
    {
        return;
    }
    ofs << isoNow() << " plan tag=" << tag
        << " succ=" << (success ? 1 : 0)
        << " topo_ms=" << topo_ms
        << " mcrrt_ms=" << mcrrt_ms
        << " opt_ms=" << opt_ms
        << " hard_ms=" << hard_ms
        << " total_ms=" << total_ms << "\n";
}
}  // namespace nmoma_planner
