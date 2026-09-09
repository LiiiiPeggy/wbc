#include <remani_real/deployment_state_machine.hpp>

namespace remani_real {

// ################################
// C++: DeploymentStateMachine implementation begin
// ################################
State DeploymentStateMachine::state() const { return state_; }

uint64_t DeploymentStateMachine::plannedCandidateId() const {
  return planned_candidate_id_;
}

CommandPermissions DeploymentStateMachine::permissions() const {
  return permissionsFor(state_);
}

bool DeploymentStateMachine::newTransactionAllowed() const {
  // Gate may accept START only outside an executing/paused frozen candidate.
  return state_ != State::Executing && state_ != State::Paused;
}

bool DeploymentStateMachine::readinessOk() const { return isReady(readiness_); }

void DeploymentStateMachine::updateReadiness(const ReadinessSnapshot& readiness) {
  readiness_ = readiness;
  const bool ready = isReady(readiness_);

  switch (state_) {
    case State::NotReady:
      if (ready) {
        state_ = State::Ready;
      }
      break;
    case State::Ready:
      if (!ready) {
        state_ = State::NotReady;
      }
      break;
    case State::Planning:
    case State::Executing:
    case State::Paused:
      if (!ready) {
        planned_candidate_id_ = 0;
        last_error_code_ = "READINESS_LOST";
        state_ = State::Error;
      }
      break;
    case State::Planned:
      if (!ready) {
        planned_candidate_id_ = 0;
        state_ = State::NotReady;
      }
      break;
    case State::Succeeded:
      if (!ready) {
        state_ = State::NotReady;
      }
      break;
    case State::Error:
      break;
  }
}

CommandResult DeploymentStateMachine::requestPlan() {
  if (!permissionsFor(state_).plan) {
    return reject("PLAN_DENIED", "plan is not permitted in current state");
  }
  if (!isReady(readiness_)) {
    return reject("NOT_READY", "readiness snapshot is incomplete");
  }
  planned_candidate_id_ = 0;
  last_error_code_.clear();
  state_ = State::Planning;
  return accept();
}

CommandResult DeploymentStateMachine::requestExecute(uint64_t candidate_id) {
  if (!permissionsFor(state_).execute) {
    return reject("EXECUTE_DENIED", "execute is not permitted in current state");
  }
  if (!isReady(readiness_)) {
    return reject("NOT_READY", "execute requires a fresh ready snapshot");
  }
  if (state_ != State::Planned || candidate_id == 0 ||
      candidate_id != planned_candidate_id_) {
    return reject("STALE_CANDIDATE", "execute candidate_id mismatch");
  }
  state_ = State::Executing;
  return accept();
}

CommandResult DeploymentStateMachine::requestPause() {
  if (!permissionsFor(state_).pause) {
    return reject("PAUSE_DENIED", "pause is not permitted in current state");
  }
  // Remain Executing until both devices confirm stopped.
  return accept();
}

CommandResult DeploymentStateMachine::requestResume(bool within_tolerance) {
  if (!permissionsFor(state_).resume) {
    return reject("RESUME_DENIED", "resume is not permitted in current state");
  }
  if (!within_tolerance) {
    return reject("RESUME_TOLERANCE", "actual state exceeds resume tolerance");
  }
  if (!isReady(readiness_)) {
    return reject("NOT_READY", "resume requires readiness");
  }
  state_ = State::Executing;
  return accept();
}

CommandResult DeploymentStateMachine::requestAbort() {
  if (!permissionsFor(state_).abort) {
    return reject("ABORT_DENIED", "abort is not permitted in current state");
  }
  if (state_ == State::Error && !isReady(readiness_)) {
    return reject("NOT_READY", "error abort requires restored health");
  }
  planned_candidate_id_ = 0;
  last_error_code_.clear();
  state_ = State::Ready;
  return accept();
}

void DeploymentStateMachine::onCandidateValidated(uint64_t candidate_id,
                                                  bool valid) {
  if (state_ != State::Planning) {
    return;
  }
  if (!valid || candidate_id == 0) {
    planned_candidate_id_ = 0;
    state_ = State::Ready;
    return;
  }
  planned_candidate_id_ = candidate_id;
  state_ = State::Planned;
}

void DeploymentStateMachine::onPlanningFailure(const std::string& error_code,
                                               bool protocol_corruption) {
  if (state_ != State::Planning) {
    return;
  }
  planned_candidate_id_ = 0;
  last_error_code_ = error_code;
  state_ = protocol_corruption ? State::Error : State::Ready;
}

void DeploymentStateMachine::onPauseConfirmed() {
  if (state_ == State::Executing) {
    state_ = State::Paused;
  }
}

void DeploymentStateMachine::onExecutionSucceeded() {
  if (state_ == State::Executing) {
    state_ = State::Succeeded;
  }
}

void DeploymentStateMachine::onExecutionFault(const std::string& error_code) {
  if (state_ == State::Executing || state_ == State::Paused) {
    planned_candidate_id_ = 0;
    last_error_code_ = error_code;
    state_ = State::Error;
  }
}

bool DeploymentStateMachine::isReady(const ReadinessSnapshot& readiness) const {
  return readiness.odom && readiness.cr10_joints && readiness.cr10_velocity &&
         readiness.tf && readiness.robot_status && readiness.action_server &&
         readiness.grid_map && readiness.ranger_watchdog;
}

CommandPermissions DeploymentStateMachine::permissionsFor(State state) const {
  CommandPermissions permissions;
  switch (state) {
    case State::NotReady:
      break;
    case State::Ready:
      permissions.plan = true;
      break;
    case State::Planning:
      permissions.abort = true;
      break;
    case State::Planned:
      permissions.plan = true;
      permissions.execute = true;
      permissions.abort = true;
      break;
    case State::Executing:
      permissions.pause = true;
      permissions.abort = true;
      break;
    case State::Paused:
      permissions.resume = true;
      permissions.abort = true;
      break;
    case State::Succeeded:
      permissions.plan = true;
      break;
    case State::Error:
      permissions.abort = true;
      break;
  }
  return permissions;
}

CommandResult DeploymentStateMachine::reject(const std::string& code,
                                             const std::string& detail) const {
  CommandResult result;
  result.accepted = false;
  result.error_code = code;
  result.detail = detail;
  return result;
}

CommandResult DeploymentStateMachine::accept() const {
  CommandResult result;
  result.accepted = true;
  return result;
}
// ################################
// C++: DeploymentStateMachine implementation end
// ################################

}  // namespace remani_real
