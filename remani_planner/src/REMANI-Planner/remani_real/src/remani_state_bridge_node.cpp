#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <remani_real_msgs/Cr10Status.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_broadcaster.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <remani_real/actual_state.hpp>
#include <remani_real/joint_state_mapper.hpp>
#include <remani_real/velocity_estimator.hpp>

namespace remani_real {
namespace {

// ################################
// C++: State Bridge display-joint config begin
// ################################
struct DisplayJointDefault {
  std::string name;
  double position{0.0};
  bool measured{false};
};

bool parseDisplayJointDefaults(const XmlRpc::XmlRpcValue& raw,
                               std::vector<DisplayJointDefault>* defaults) {
  if (raw.getType() != XmlRpc::XmlRpcValue::TypeArray || raw.size() == 0) {
    return false;
  }
  defaults->clear();
  defaults->reserve(static_cast<std::size_t>(raw.size()));
  for (int index = 0; index < raw.size(); ++index) {
    XmlRpc::XmlRpcValue entry = raw[index];
    if (entry.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
        !entry.hasMember("name") || !entry.hasMember("position")) {
      return false;
    }
    DisplayJointDefault joint;
    joint.name = static_cast<std::string>(entry["name"]);
    joint.position = static_cast<double>(entry["position"]);
    joint.measured = entry.hasMember("measured")
                         ? static_cast<bool>(entry["measured"])
                         : false;
    if (joint.name.empty() || !std::isfinite(joint.position) || joint.measured) {
      return false;
    }
    defaults->push_back(joint);
  }
  return !defaults->empty();
}
// ################################
// C++: State Bridge display-joint config end
// ################################

}  // namespace

class RemaniStateBridgeNode {
 public:
  // ################################
  // C++: RemaniStateBridgeNode ctor begin
  // ################################
  RemaniStateBridgeNode()
      : private_nh_("~"),
        velocity_estimator_(3, 3.0, 0.5, 0.2) {
    int min_samples = 3;
    double max_abs_velocity = 3.0;
    double alpha = 0.5;
    double max_dt = 0.2;
    ros::NodeHandle velocity_nh(nh_, "state_bridge/velocity_estimator");
    velocity_nh.param("min_samples", min_samples, min_samples);
    velocity_nh.param("max_abs_velocity", max_abs_velocity, max_abs_velocity);
    velocity_nh.param("alpha", alpha, alpha);
    velocity_nh.param("max_dt", max_dt, max_dt);
    nh_.param("state_bridge/status_stale_timeout", status_stale_timeout_,
              status_stale_timeout_);

    velocity_estimator_ = VelocityEstimator(
        static_cast<std::size_t>(std::max(3, min_samples)), max_abs_velocity,
        alpha, max_dt);

    XmlRpc::XmlRpcValue raw_defaults;
    if (!ros::param::get("/state_bridge/display_joint_defaults", raw_defaults) &&
        !nh_.getParam("state_bridge/display_joint_defaults", raw_defaults) &&
        !private_nh_.getParam("display_joint_defaults", raw_defaults)) {
      throw std::runtime_error("state_bridge display_joint_defaults is required");
    }
    if (!parseDisplayJointDefaults(raw_defaults, &display_defaults_)) {
      throw std::runtime_error("state_bridge display_joint_defaults is invalid");
    }

    planning_pub_ =
        nh_.advertise<sensor_msgs::JointState>("/remani/cr10_joint_states", 10);
    robot_model_pub_ =
        nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
    raw_sub_ = nh_.subscribe("/remani/cr10_joint_states_raw", 10,
                             &RemaniStateBridgeNode::onRawJoints, this);
    odom_sub_ = nh_.subscribe("/odom", 10, &RemaniStateBridgeNode::onOdom, this);
    status_sub_ = nh_.subscribe("/remani/cr10_status", 10,
                                &RemaniStateBridgeNode::onStatus, this);
  }
  // ################################
  // C++: RemaniStateBridgeNode ctor end
  // ################################

 private:
  // ################################
  // C++: State Bridge callbacks begin
  // ################################
  void onStatus(const remani_real_msgs::Cr10Status::ConstPtr& msg) {
    latest_status_ = *msg;
    have_status_ = true;
    snapshot_.robot_connected = msg->connected;
    snapshot_.robot_enabled = msg->enabled;
    snapshot_.robot_fault = msg->error_status != 0;
  }

  bool statusFresh(const ros::Time& now) const {
    if (!have_status_ || latest_status_.header.stamp.isZero()) {
      return false;
    }
    const double age = (now - latest_status_.header.stamp).toSec();
    return std::isfinite(age) && age >= 0.0 && age <= status_stale_timeout_;
  }

  void onOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    if (msg->header.frame_id != "world" ||
        msg->child_frame_id != "base_link" ||
        !std::isfinite(msg->pose.pose.position.x) ||
        !std::isfinite(msg->pose.pose.position.y) ||
        !std::isfinite(msg->pose.pose.orientation.x) ||
        !std::isfinite(msg->pose.pose.orientation.y) ||
        !std::isfinite(msg->pose.pose.orientation.z) ||
        !std::isfinite(msg->pose.pose.orientation.w)) {
      snapshot_.odom_valid = false;
      snapshot_.tf_valid = false;
      return;
    }

    geometry_msgs::TransformStamped transform;
    transform.header = msg->header;
    transform.header.frame_id = "world";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = msg->pose.pose.position.x;
    transform.transform.translation.y = msg->pose.pose.position.y;
    transform.transform.translation.z = msg->pose.pose.position.z;
    transform.transform.rotation = msg->pose.pose.orientation;
    tf_broadcaster_.sendTransform(transform);

    snapshot_.ros_stamp = msg->header.stamp;
    snapshot_.base_xy << msg->pose.pose.position.x, msg->pose.pose.position.y;
    snapshot_.odom_valid = true;
    snapshot_.tf_valid = true;
  }

  void onRawJoints(const sensor_msgs::JointState::ConstPtr& msg) {
    const MappedJointState mapped = JointStateMapper::map(*msg);
    if (!mapped.valid) {
      snapshot_.joints_valid = false;
      return;
    }

    Eigen::Matrix<double, 6, 1> q;
    for (int index = 0; index < 6; ++index) {
      q(index) = mapped.position[static_cast<std::size_t>(index)];
    }
    const VelocityEstimate velocity =
        velocity_estimator_.update(msg->header.stamp, q);

    sensor_msgs::JointState planning = mapped.planning_msg;
    if (velocity.valid) {
      planning.velocity.resize(6);
      for (int index = 0; index < 6; ++index) {
        planning.velocity[static_cast<std::size_t>(index)] = velocity.qd(index);
      }
      snapshot_.qd = velocity.qd;
      snapshot_.velocity_valid = true;
    } else {
      planning.velocity.clear();
      snapshot_.qd.setZero();
      snapshot_.velocity_valid = false;
    }

    snapshot_.q = q;
    snapshot_.joints_valid = true;
    snapshot_.ros_stamp = msg->header.stamp;
    // Formal readiness still requires fresh normalized status; do not invent it.
    if (!statusFresh(ros::Time::now())) {
      snapshot_.robot_connected = false;
      snapshot_.robot_enabled = false;
      snapshot_.robot_fault = true;
    }

    planning_pub_.publish(planning);

    sensor_msgs::JointState robot_model;
    robot_model.header = planning.header;
    robot_model.name = planning.name;
    robot_model.position = planning.position;
    if (velocity.valid) {
      robot_model.velocity = planning.velocity;
    }
    for (const DisplayJointDefault& joint : display_defaults_) {
      robot_model.name.push_back(joint.name);
      robot_model.position.push_back(joint.position);
      if (velocity.valid) {
        robot_model.velocity.push_back(0.0);
      }
    }
    robot_model_pub_.publish(robot_model);
  }
  // ################################
  // C++: State Bridge callbacks end
  // ################################

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber raw_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber status_sub_;
  ros::Publisher planning_pub_;
  ros::Publisher robot_model_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  VelocityEstimator velocity_estimator_;
  std::vector<DisplayJointDefault> display_defaults_;
  ActualStateSnapshot snapshot_;
  remani_real_msgs::Cr10Status latest_status_;
  bool have_status_{false};
  double status_stale_timeout_{0.5};
};

}  // namespace remani_real

int main(int argc, char** argv) {
  // ################################
  // C++: remani_state_bridge_node main begin
  // ################################
  ros::init(argc, argv, "remani_state_bridge_node");
  try {
    remani_real::RemaniStateBridgeNode node;
    ros::spin();
  } catch (const std::exception& ex) {
    ROS_FATAL("remani_state_bridge_node failed: %s", ex.what());
    return 1;
  }
  return 0;
  // ################################
  // C++: remani_state_bridge_node main end
  // ################################
}
