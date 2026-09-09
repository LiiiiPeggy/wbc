#include <remani_real/preview_publisher.hpp>

#include <cmath>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/ColorRGBA.h>

namespace remani_real {
namespace {

// ################################
// C++: PreviewPublisher helpers begin
// ################################
constexpr double kPathDt = 0.05;
constexpr double kRobotDt = 0.25;

std_msgs::ColorRGBA makeColor(float r, float g, float b, float a = 1.0f) {
  std_msgs::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

std_msgs::ColorRGBA singulColor(int singul) {
  if (singul < 0) {
    return makeColor(0.2f, 0.4f, 1.0f);
  }
  return makeColor(0.1f, 0.8f, 0.3f);
}
// ################################
// C++: PreviewPublisher helpers end
// ################################

}  // namespace

// ################################
// C++: PreviewPublisher implementation begin
// ################################
PreviewPublisher::PreviewPublisher(ros::NodeHandle& nh) {
  advertised_topics_ = {
      "/remani/candidate_robot",
      "/remani/candidate_base_path",
      "/remani/candidate_ee_path",
  };
  robot_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
      advertised_topics_[0], 1, true);
  base_path_pub_ =
      nh.advertise<nav_msgs::Path>(advertised_topics_[1], 1, true);
  ee_path_pub_ = nh.advertise<nav_msgs::Path>(advertised_topics_[2], 1, true);
}

void PreviewPublisher::publish(const FrozenCandidate& candidate,
                               const ValidationReport& report,
                               const ValidationEnvironment* environment) {
  if (!candidate) {
    return;
  }

  const std::string ns = "candidate_" + std::to_string(candidate->id());
  const ros::Time stamp = candidate->rawTransactionStamp().isZero()
                              ? ros::Time::now()
                              : candidate->rawTransactionStamp();

  nav_msgs::Path base_path;
  base_path.header.stamp = stamp;
  base_path.header.frame_id = "map";
  nav_msgs::Path ee_path;
  ee_path.header = base_path.header;

  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker clear;
  clear.action = visualization_msgs::Marker::DELETEALL;
  markers.markers.push_back(clear);

  const double duration = candidate->duration();
  const bool rejected = !report.valid;
  int marker_id = 0;

  auto appendSample = [&](double t, bool emit_robot) {
    const WholeBodySample sample = candidate->sample(t);
    geometry_msgs::PoseStamped base_pose;
    base_pose.header = base_path.header;
    base_pose.pose.position.x = sample.position(0);
    base_pose.pose.position.y = sample.position(1);
    base_pose.pose.position.z = 0.0;
    base_pose.pose.orientation.z = std::sin(0.5 * sample.base_yaw);
    base_pose.pose.orientation.w = std::cos(0.5 * sample.base_yaw);
    base_path.poses.push_back(base_pose);

    if (environment != nullptr) {
      Eigen::Vector3d car;
      car << sample.position(0), sample.position(1), sample.base_yaw;
      const Eigen::Matrix4d ee =
          environment->eePose(car, sample.position.segment(2, 6));
      geometry_msgs::PoseStamped ee_pose;
      ee_pose.header = ee_path.header;
      ee_pose.pose.position.x = ee(0, 3);
      ee_pose.pose.position.y = ee(1, 3);
      ee_pose.pose.position.z = ee(2, 3);
      ee_path.poses.push_back(ee_pose);
    }

    if (!emit_robot) {
      return;
    }
    visualization_msgs::Marker marker;
    marker.header = base_path.header;
    marker.ns = ns;
    marker.id = marker_id++;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = base_pose.pose;
    marker.scale.x = 0.35;
    marker.scale.y = 0.08;
    marker.scale.z = 0.08;
    marker.color = rejected ? makeColor(1.0f, 0.1f, 0.1f)
                            : singulColor(sample.singul);
    markers.markers.push_back(marker);
  };

  if (duration > 0.0) {
    for (double t = 0.0; t < duration; t += kPathDt) {
      appendSample(t, false);
    }
    appendSample(duration, false);

    for (double t = 0.0; t < duration; t += kRobotDt) {
      appendSample(t, true);
    }
    appendSample(duration, true);
  }

  if (rejected) {
    visualization_msgs::Marker diag;
    diag.header = base_path.header;
    diag.ns = ns + "_reject";
    diag.id = marker_id++;
    diag.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    diag.action = visualization_msgs::Marker::ADD;
    if (!base_path.poses.empty()) {
      diag.pose = base_path.poses.front().pose;
    } else if (candidate->duration() > 0.0) {
      const WholeBodySample start = candidate->sample(0.0);
      diag.pose.position.x = start.position(0);
      diag.pose.position.y = start.position(1);
      diag.pose.orientation.w = 1.0;
    } else {
      diag.pose.orientation.w = 1.0;
    }
    diag.pose.position.z = 0.6;
    diag.scale.z = 0.2;
    diag.color = makeColor(1.0f, 0.1f, 0.1f);
    diag.text = report.error_code.empty() ? "REJECTED" : report.error_code;
    markers.markers.push_back(diag);
  }

  robot_pub_.publish(markers);
  base_path_pub_.publish(base_path);
  ee_path_pub_.publish(ee_path);
}

const std::vector<std::string>& PreviewPublisher::advertisedTopics() const {
  return advertised_topics_;
}
// ################################
// C++: PreviewPublisher implementation end
// ################################

}  // namespace remani_real
