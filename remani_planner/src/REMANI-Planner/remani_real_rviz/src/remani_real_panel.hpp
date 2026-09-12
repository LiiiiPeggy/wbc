#ifndef REMANI_REAL_RVIZ_PANEL_H
#define REMANI_REAL_RVIZ_PANEL_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#include <remani_real_msgs/ExecutionState.h>
#include <remani_real_msgs/ExecuteCandidate.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Trigger.h>
#endif

#include <QWidget>

class QPushButton;
class QLabel;

namespace remani_real_rviz {

// ################################
// C++: RemaniRealPanel RViz widget begin
// ################################
class RemaniRealPanel : public rviz::Panel {
  Q_OBJECT
 public:
  explicit RemaniRealPanel(QWidget* parent = nullptr);

 protected Q_SLOTS:
  void onPlan();
  void onExecute();
  void onPause();
  void onResume();
  void onAbort();

 private:
  void onExecutionState(const remani_real_msgs::ExecutionState::ConstPtr& msg);
  void applyView();
  void setBusy(bool busy);

  ros::NodeHandle nh_;
  ros::Subscriber state_sub_;
  ros::Publisher plan_pub_;
  ros::ServiceClient execute_client_;
  ros::ServiceClient pause_client_;
  ros::ServiceClient resume_client_;
  ros::ServiceClient abort_client_;

  remani_real_msgs::ExecutionState latest_state_;
  bool have_state_{false};
  bool click_busy_{false};
  uint8_t busy_planner_state_{0};
  uint8_t busy_executor_state_{0};

  QPushButton* plan_button_{nullptr};
  QPushButton* execute_button_{nullptr};
  QPushButton* pause_button_{nullptr};
  QPushButton* resume_button_{nullptr};
  QPushButton* abort_button_{nullptr};
  QLabel* warning_label_{nullptr};
  QLabel* status_label_{nullptr};
};
// ################################
// C++: RemaniRealPanel RViz widget end
// ################################

}  // namespace remani_real_rviz

#endif
