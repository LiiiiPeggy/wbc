#include "remani_real_panel.hpp"

#include <remani_real_rviz/panel_view_model.hpp>

#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.h>

namespace remani_real_rviz {

// ################################
// C++: RemaniRealPanel implementation begin
// ################################
RemaniRealPanel::RemaniRealPanel(QWidget* parent) : rviz::Panel(parent) {
  plan_button_ = new QPushButton("Plan");
  execute_button_ = new QPushButton("Execute");
  pause_button_ = new QPushButton("Pause");
  resume_button_ = new QPushButton("Resume");
  abort_button_ = new QPushButton("Abort");
  warning_label_ = new QLabel;
  status_label_ = new QLabel("waiting for /remani/execution_state");

  warning_label_->setWordWrap(true);
  status_label_->setWordWrap(true);

  QHBoxLayout* buttons = new QHBoxLayout;
  buttons->addWidget(plan_button_);
  buttons->addWidget(execute_button_);
  buttons->addWidget(pause_button_);
  buttons->addWidget(resume_button_);
  buttons->addWidget(abort_button_);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(warning_label_);
  layout->addLayout(buttons);
  layout->addWidget(status_label_);
  setLayout(layout);

  connect(plan_button_, SIGNAL(clicked()), this, SLOT(onPlan()));
  connect(execute_button_, SIGNAL(clicked()), this, SLOT(onExecute()));
  connect(pause_button_, SIGNAL(clicked()), this, SLOT(onPause()));
  connect(resume_button_, SIGNAL(clicked()), this, SLOT(onResume()));
  connect(abort_button_, SIGNAL(clicked()), this, SLOT(onAbort()));

  state_sub_ = nh_.subscribe("/remani/execution_state", 1,
                             &RemaniRealPanel::onExecutionState, this);
  plan_pub_ = nh_.advertise<std_msgs::Empty>("/ee_goal_plan", 1, false);
  execute_client_ =
      nh_.serviceClient<remani_real_msgs::ExecuteCandidate>("/remani/execute");
  pause_client_ = nh_.serviceClient<std_srvs::Trigger>("/remani/pause");
  resume_client_ = nh_.serviceClient<std_srvs::Trigger>("/remani/resume");
  abort_client_ = nh_.serviceClient<std_srvs::Trigger>("/remani/abort");

  applyView();
}

void RemaniRealPanel::setBusy(bool busy) {
  click_busy_ = busy;
  if (busy) {
    // ################################
    // C++: latch published state so busy clears only after a real transition
    // ################################
    busy_planner_state_ = latest_state_.planner_state;
    busy_executor_state_ = latest_state_.executor_state;
  }
  applyView();
}

void RemaniRealPanel::applyView() {
  PanelViewModel view;
  if (have_state_) {
    view = PanelViewModel::from(latest_state_);
  }

  const bool allow = !click_busy_;
  plan_button_->setEnabled(allow && view.plan_enabled);
  execute_button_->setEnabled(allow && view.execute_enabled);
  pause_button_->setEnabled(allow && view.pause_enabled);
  resume_button_->setEnabled(allow && view.resume_enabled);
  // Abort stays available whenever the published state permits it, even while
  // another command is waiting for the next ExecutionState update.
  abort_button_->setEnabled(view.abort_enabled);

  warning_label_->setText(QString::fromStdString(view.environment_warning));
  if (!have_state_) {
    status_label_->setText("waiting for /remani/execution_state");
    return;
  }

  QString status = QString("candidate=%1 complete=%2 valid=%3 progress=%4")
                       .arg(latest_state_.candidate_id)
                       .arg(latest_state_.candidate_complete ? "true" : "false")
                       .arg(latest_state_.candidate_valid ? "true" : "false")
                       .arg(latest_state_.execution_progress, 0, 'f', 3);
  if (!latest_state_.last_error_code.empty()) {
    status += QString(" error=%1")
                  .arg(QString::fromStdString(latest_state_.last_error_code));
  }
  status_label_->setText(status);
}

void RemaniRealPanel::onExecutionState(
    const remani_real_msgs::ExecutionState::ConstPtr& msg) {
  latest_state_ = *msg;
  have_state_ = true;
  // ################################
  // C++: clear click debounce only after planner/executor state changes begin
  // ################################
  if (click_busy_ &&
      (msg->planner_state != busy_planner_state_ ||
       msg->executor_state != busy_executor_state_)) {
    click_busy_ = false;
  }
  applyView();
}

void RemaniRealPanel::onPlan() {
  setBusy(true);
  std_msgs::Empty empty;
  plan_pub_.publish(empty);
}

void RemaniRealPanel::onExecute() {
  setBusy(true);
  remani_real_msgs::ExecuteCandidate service;
  service.request.candidate_id = latest_state_.candidate_id;
  if (!execute_client_.call(service) || !service.response.accepted) {
    click_busy_ = false;
    applyView();
  }
}

void RemaniRealPanel::onPause() {
  setBusy(true);
  std_srvs::Trigger service;
  if (!pause_client_.call(service) || !service.response.success) {
    click_busy_ = false;
    applyView();
  }
}

void RemaniRealPanel::onResume() {
  setBusy(true);
  std_srvs::Trigger service;
  if (!resume_client_.call(service) || !service.response.success) {
    click_busy_ = false;
    applyView();
  }
}

void RemaniRealPanel::onAbort() {
  setBusy(true);
  std_srvs::Trigger service;
  if (!abort_client_.call(service) || !service.response.success) {
    click_busy_ = false;
    applyView();
  }
}
// ################################
// C++: RemaniRealPanel implementation end
// ################################

}  // namespace remani_real_rviz

PLUGINLIB_EXPORT_CLASS(remani_real_rviz::RemaniRealPanel, rviz::Panel)
