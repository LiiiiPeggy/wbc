#pragma once

#include <stdexcept>
#include <string>

#include <ros/ros.h>

namespace remani_planner {

enum class RuntimeMode { Sim, Real };

enum class ExecutionOwner { Internal, External };

class ExecutionPolicy {
 public:
  static ExecutionPolicy fromStrings(const std::string& mode,
                                     const std::string& owner) {
    if (mode == "sim" && owner == "internal") {
      return ExecutionPolicy(RuntimeMode::Sim, ExecutionOwner::Internal);
    }
    if (mode == "real" && owner == "external") {
      return ExecutionPolicy(RuntimeMode::Real, ExecutionOwner::External);
    }
    throw std::invalid_argument("invalid mode/execution_owner combination");
  }

  static ExecutionPolicy load(const ros::NodeHandle& nh) {
    std::string mode;
    std::string owner;
    nh.param("mode", mode, std::string("sim"));
    nh.param("execution_owner", owner, std::string("internal"));
    return fromStrings(mode, owner);
  }

  bool ownsInternalExecution() const {
    return owner_ == ExecutionOwner::Internal;
  }

  bool isRealPlanOnly() const {
    return mode_ == RuntimeMode::Real && owner_ == ExecutionOwner::External;
  }

  RuntimeMode mode() const { return mode_; }

  ExecutionOwner owner() const { return owner_; }

 private:
  ExecutionPolicy(RuntimeMode mode, ExecutionOwner owner)
      : mode_(mode), owner_(owner) {}

  RuntimeMode mode_;
  ExecutionOwner owner_;
};

}  // namespace remani_planner
