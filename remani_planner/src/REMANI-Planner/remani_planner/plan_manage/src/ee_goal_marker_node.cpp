// ################################
// C++: EE interactive marker UI node begin
// ################################
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Empty.h>
#include <ros/ros.h>
#include <boost/bind.hpp>

#include <cmath>
#include <memory>
#include <string>

namespace
{

// ################################
// C++: normalize quaternion — RViz rejects/hides markers with non-unit quats
// ################################
geometry_msgs::Quaternion normalizedQuat(double x, double y, double z, double w)
{
  const double n = std::sqrt(x * x + y * y + z * z + w * w);
  geometry_msgs::Quaternion q;
  if(n < 1e-9){
    q.x = 0.0;
    q.y = 0.0;
    q.z = 0.0;
    q.w = 1.0;
    return q;
  }
  q.x = x / n;
  q.y = y / n;
  q.z = z / n;
  q.w = w / n;
  return q;
}

geometry_msgs::Quaternion normalizedQuat(const geometry_msgs::Quaternion &qin)
{
  return normalizedQuat(qin.x, qin.y, qin.z, qin.w);
}
// ################################

class EeGoalMarkerNode
{
public:
  explicit EeGoalMarkerNode(ros::NodeHandle &nh)
      : nh_(nh),
        server_(new interactive_markers::InteractiveMarkerServer("ee_goal_marker")),
        have_current_pose_(false),
        marker_inserted_(false),
        logged_waiting_(false),
        logged_pose_received_(false)
  {
    ee_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/ee_goal", 1);
    ee_current_sub_ = nh_.subscribe("/ee_current_pose", 1,
                                    &EeGoalMarkerNode::currentPoseCallback, this);
    // ################################
    // C++: explicit plan trigger — drag does not publish /ee_goal
    // ################################
    ee_goal_plan_sub_ = nh_.subscribe("/ee_goal_plan", 1,
                                      &EeGoalMarkerNode::planCmdCallback, this);
    // ################################

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
        if(!logged_waiting_){
          ROS_INFO("[EE MARKER] waiting current pose");
          logged_waiting_ = true;
        }
      }else if(!marker_inserted_){
        insertMarker(latest_pose_);
        marker_inserted_ = true;
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
    // ################################
    // C++: log first /ee_current_pose then create marker once
    // ################################
    if(!logged_pose_received_){
      ROS_INFO("[EE MARKER] current pose received");
      logged_pose_received_ = true;
    }
    // ################################
  }

  // ################################
  // C++: /ee_goal_plan → publish cached marker pose as /ee_goal
  // ################################
  void planCmdCallback(const std_msgs::EmptyConstPtr & /*msg*/)
  {
    ROS_INFO("[EE MARKER] /ee_goal_plan received");
    if(!marker_inserted_){
      ROS_WARN("[EE MARKER] plan ignored: marker not ready");
      return;
    }
    publishGoal(marker_pose_);
  }
  // ################################

  void updateCachedMarkerPose(const geometry_msgs::Pose &pose)
  {
    marker_pose_ = pose;
    marker_pose_.orientation = normalizedQuat(marker_pose_.orientation);
  }

  visualization_msgs::InteractiveMarker createInteractiveMarker(
      const geometry_msgs::PoseStamped &pose) const
  {
    // ################################
    // C++: visible SPHERE + unit-quat 6DOF axes; no empty TEXT_VIEW_FACING
    // ################################
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "world";
    int_marker.header.stamp = ros::Time::now();
    int_marker.name = "ee_goal";
    int_marker.description = "Right-click: Plan";
    int_marker.pose = pose.pose;
    int_marker.pose.orientation = normalizedQuat(pose.pose.orientation);
    // ################################
    // C++: larger marker scale for easier RViz Interact picking
    // ################################
    int_marker.scale = 0.45;

    visualization_msgs::Marker sphere;
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.pose.orientation.w = 1.0;
    // Larger pick target (was 0.08); easier to click in RViz.
    sphere.scale.x = 0.18;
    sphere.scale.y = 0.18;
    sphere.scale.z = 0.18;
    // ################################
    sphere.color.r = 1.0;
    sphere.color.g = 0.85;
    sphere.color.b = 0.0;
    sphere.color.a = 1.0;

    visualization_msgs::InteractiveMarkerControl sphere_control;
    sphere_control.always_visible = true;
    sphere_control.orientation = normalizedQuat(0.0, 0.0, 0.0, 1.0);
    sphere_control.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
    sphere_control.name = "ee_goal_sphere";
    sphere_control.markers.push_back(sphere);
    int_marker.controls.push_back(sphere_control);

    visualization_msgs::InteractiveMarkerControl axis;
    axis.orientation_mode =
        visualization_msgs::InteractiveMarkerControl::INHERIT;
    axis.always_visible = true;

    // X: rotate + move (unit quat ≈ normalize(1,1,0,0))
    axis.orientation = normalizedQuat(1.0, 0.0, 0.0, 1.0);
    axis.name = "rotate_x";
    axis.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(axis);
    axis.name = "move_x";
    axis.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(axis);

    // Z: rotate + move (unit quat ≈ normalize(0,1,0,1))
    axis.orientation = normalizedQuat(0.0, 1.0, 0.0, 1.0);
    axis.name = "rotate_z";
    axis.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(axis);
    axis.name = "move_z";
    axis.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(axis);

    // Y: rotate + move (unit quat ≈ normalize(0,0,1,1))
    axis.orientation = normalizedQuat(0.0, 0.0, 1.0, 1.0);
    axis.name = "rotate_y";
    axis.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(axis);
    axis.name = "move_y";
    axis.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(axis);
    // ################################

    return int_marker;
  }

  void insertMarker(const geometry_msgs::PoseStamped &pose)
  {
    ROS_INFO("[EE MARKER] create marker");
    visualization_msgs::InteractiveMarker int_marker =
        createInteractiveMarker(pose);

    // ################################
    // C++: seed cached pose; only Plan /ee_goal_plan publishes /ee_goal
    // ################################
    updateCachedMarkerPose(int_marker.pose);
    // ################################

    ROS_INFO("[EE MARKER] insert marker");
    server_->insert(int_marker);
    server_->setCallback(int_marker.name,
                         boost::bind(&EeGoalMarkerNode::processFeedback, this, _1));
    menu_handler_.apply(*server_, int_marker.name);

    ROS_INFO("[EE MARKER] applyChanges");
    server_->applyChanges();
  }

  void processFeedback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    // ################################
    // C++: cache pose on drag; publish /ee_goal only on Plan menu
    // ################################
    if(feedback->event_type ==
           visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE ||
       feedback->event_type ==
           visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN ||
       feedback->event_type ==
           visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP){
      updateCachedMarkerPose(feedback->pose);
      if(feedback->event_type ==
         visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP){
        ROS_INFO_THROTTLE(1.0, "[EE MARKER] pose adjusted (no plan)");
      }
      return;
    }

    if(feedback->event_type ==
           visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT){
      if(feedback->menu_entry_id == 1){
        updateCachedMarkerPose(feedback->pose);
        ROS_INFO("[EE MARKER] Plan confirmed → /ee_goal");
        publishGoal(marker_pose_);
      }else if(feedback->menu_entry_id == 2){
        if(have_current_pose_){
          geometry_msgs::Pose reset_pose = latest_pose_.pose;
          reset_pose.orientation = normalizedQuat(reset_pose.orientation);
          updateCachedMarkerPose(reset_pose);
          server_->setPose("ee_goal", reset_pose);
          server_->applyChanges();
        }
      }
      return;
    }
    // ################################
  }

  void publishGoal(const geometry_msgs::Pose &pose)
  {
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    msg.pose = pose;
    msg.pose.orientation = normalizedQuat(msg.pose.orientation);
    ee_goal_pub_.publish(msg);
    ROS_INFO("[EE MARKER] published /ee_goal");
  }

  ros::NodeHandle nh_;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  interactive_markers::MenuHandler menu_handler_;
  ros::Publisher ee_goal_pub_;
  ros::Subscriber ee_current_sub_;
  ros::Subscriber ee_goal_plan_sub_;
  geometry_msgs::PoseStamped latest_pose_;
  geometry_msgs::Pose marker_pose_;
  bool have_current_pose_;
  bool marker_inserted_;
  bool logged_waiting_;
  bool logged_pose_received_;
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
