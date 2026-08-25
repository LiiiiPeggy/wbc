// ################################
// C++: EE interactive marker UI node begin
// ################################
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <boost/bind.hpp>

#include <memory>
#include <string>

namespace
{

class EeGoalMarkerNode
{
public:
  explicit EeGoalMarkerNode(ros::NodeHandle &nh)
      : nh_(nh),
        server_(new interactive_markers::InteractiveMarkerServer("ee_goal_marker")),
        have_current_pose_(false),
        marker_inserted_(false)
  {
    ee_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/ee_goal", 1);
    ee_current_sub_ = nh_.subscribe("/ee_current_pose", 1,
                                    &EeGoalMarkerNode::currentPoseCallback, this);

    menu_handler_.insert("Plan",
                         boost::bind(&EeGoalMarkerNode::processFeedback, this, _1));
    menu_handler_.insert("Reset",
                         boost::bind(&EeGoalMarkerNode::processFeedback, this, _1));
  }

  void spin()
  {
    ros::Rate rate(20.0);
    while(ros::ok()){
      if(!have_current_pose_){
        ROS_INFO_THROTTLE(2.0, "[ee_goal_marker] waiting for current EE pose");
      }else if(!marker_inserted_){
        insertMarker(latest_pose_);
        marker_inserted_ = true;
        ROS_INFO("[ee_goal_marker] inserted 6DOF marker at current EE pose");
      }
      ros::spinOnce();
      rate.sleep();
    }
  }

private:
  void currentPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
  {
    latest_pose_ = *msg;
    have_current_pose_ = true;
  }

  void insertMarker(const geometry_msgs::PoseStamped &pose)
  {
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "world";
    int_marker.header.stamp = ros::Time::now();
    int_marker.name = "ee_goal";
    int_marker.description = "EE 6D Goal";
    int_marker.pose = pose.pose;
    int_marker.scale = 0.25;

    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = true;
    control.orientation_mode =
        visualization_msgs::InteractiveMarkerControl::INHERIT;
    control.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
    control.name = "move_rotate_3d";
    int_marker.controls.push_back(control);

    visualization_msgs::InteractiveMarkerControl menu_control;
    menu_control.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::MENU;
    menu_control.name = "menu";
    int_marker.controls.push_back(menu_control);

    server_->insert(int_marker);
    server_->setCallback(int_marker.name,
                         boost::bind(&EeGoalMarkerNode::processFeedback, this, _1));
    menu_handler_.apply(*server_, int_marker.name);
    server_->applyChanges();
  }

  void processFeedback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    if(feedback->event_type ==
           visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT){
      if(feedback->menu_entry_id == 1){
        publishGoal(feedback->pose);
      }else if(feedback->menu_entry_id == 2){
        if(have_current_pose_){
          server_->setPose("ee_goal", latest_pose_.pose);
          server_->applyChanges();
        }
      }
      return;
    }

    if(feedback->event_type ==
       visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP){
      publishGoal(feedback->pose);
    }
  }

  void publishGoal(const geometry_msgs::Pose &pose)
  {
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    msg.pose = pose;
    ee_goal_pub_.publish(msg);
    ROS_INFO("[ee_goal_marker] published /ee_goal");
  }

  ros::NodeHandle nh_;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  interactive_markers::MenuHandler menu_handler_;
  ros::Publisher ee_goal_pub_;
  ros::Subscriber ee_current_sub_;
  geometry_msgs::PoseStamped latest_pose_;
  bool have_current_pose_;
  bool marker_inserted_;
};

}  // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ee_goal_marker_node");
  ros::NodeHandle nh;
  EeGoalMarkerNode node(nh);
  node.spin();
  return 0;
}
// ################################
// C++: EE interactive marker UI node end
// ################################
