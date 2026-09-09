#pragma once

#include <cstdint>
#include <string>

namespace remani_real {

// ################################
// C++: DeploymentStateMachine contract begin
// ################################
enum class State : uint8_t {
  NotReady,
  Ready,
  Planning,
  Planned,
  Executing,
  Paused,
  Succeeded,
  Error
};

struct CommandPermissions {
  bool plan{false};
  bool execute{false};
  bool pause{false};
  bool resume{false};
  bool abort{false};
};

struct ReadinessSnapshot {
  bool odom{false};
  bool cr10_joints{false};
  bool cr10_velocity{false};
  bool tf{false};
  bool robot_status{false};
  bool action_server{false};
  bool grid_map{false};
  bool ranger_watchdog{false};
};

struct CommandResult {
  bool accepted{false};
  std::string error_code;
  std::string detail;
};

class DeploymentStateMachine {
 public:
  State state() const;
  uint64_t plannedCandidateId() const;
  CommandPermissions permissions() const;
  bool newTransactionAllowed() const;
  bool readinessOk() const;

  void updateReadiness(const ReadinessSnapshot& readiness);
  CommandResult requestPlan();
  CommandResult requestExecute(uint64_t candidate_id);
  CommandResult requestPause();
  CommandResult requestResume(bool within_tolerance);
  CommandResult requestAbort();

  void onCandidateValidated(uint64_t candidate_id, bool valid);
  void onPlanningFailure(const std::string& error_code, bool protocol_corruption);
  void onPauseConfirmed();
  void onExecutionSucceeded();
  void onExecutionFault(const std::string& error_code);

 private:
  bool isReady(const ReadinessSnapshot& readiness) const;
  CommandPermissions permissionsFor(State state) const;
  CommandResult reject(const std::string& code, const std::string& detail) const;
  CommandResult accept() const;

  State state_{State::NotReady};
  ReadinessSnapshot readiness_;
  uint64_t planned_candidate_id_{0};
  std::string last_error_code_;
};
// ################################
// C++: DeploymentStateMachine contract end
// ################################

}  // namespace remani_real
