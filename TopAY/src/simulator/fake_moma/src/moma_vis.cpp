#include <iostream>
#include <math.h>
#include <random>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include "fake_moma/MomaState.h"
#include "fake_moma/moma_param.h"

using namespace std;

ros::Subscriber state_sub;
ros::Publisher marker_pub, cylinder_pub, sphere_pub;

visualization_msgs::MarkerArray moma_marker;
visualization_msgs::MarkerArray colli_marker;

Eigen::Vector3d now_se2 = Eigen::Vector3d::Zero();
vector<float> now_q;
MomaParam moma_param;

// ################################
// C++: Pose profile meshes from typed links and YAML visual offsets
// ################################
Eigen::Matrix4d meshLinkTransform(const KinematicResult& links, const MeshPart& part)
{
	switch (part.role)
	{
		case MeshRole::Base:
			return links.base_T;
		case MeshRole::ArmBase:
			return links.arm_base_T;
		case MeshRole::ArmLink:
			return links.arm_link_T.at(part.index);
		case MeshRole::Ag95:
			return links.ee_T;
	}
	throw std::runtime_error("Unknown profile mesh role");
}

geometry_msgs::Pose poseFromTransform(const Eigen::Matrix4d& transform)
{
	geometry_msgs::Pose pose;
	const Eigen::Quaterniond orientation(transform.block<3, 3>(0, 0));
	pose.position.x = transform(0, 3);
	pose.position.y = transform(1, 3);
	pose.position.z = transform(2, 3);
	pose.orientation.w = orientation.w();
	pose.orientation.x = orientation.x();
	pose.orientation.y = orientation.y();
	pose.orientation.z = orientation.z();
	return pose;
}

void updateMeshMarkers(const Eigen::VectorXd& state)
{
	const KinematicResult links = moma_param.getLinkTransforms(state);
	for (size_t index = 0; index < moma_param.mesh_parts.size(); ++index)
	{
		const MeshPart& part = moma_param.mesh_parts[index];
		moma_marker.markers[index].header.stamp = ros::Time::now();
		moma_marker.markers[index].pose =
			poseFromTransform(meshLinkTransform(links, part) * part.link_T_visual);
	}
}

void initParams()
{
	// ################################
	// C++: Create marker instances exclusively from profile mesh parts
	// ################################
	now_q.assign(moma_param.dof_num, 0.0);
	for (size_t index = 0; index < moma_param.mesh_parts.size(); ++index)
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = "world";
		marker.id = static_cast<int>(index);
		marker.type = visualization_msgs::Marker::MESH_RESOURCE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.color.a = 1.0;
		marker.color.r = marker.color.g = marker.color.b = 0.5;
		// ################################
		// C++: Apply YAML mesh scale (AG95 STL is millimetres)
		// ################################
		marker.scale.x = moma_param.mesh_parts[index].scale.x();
		marker.scale.y = moma_param.mesh_parts[index].scale.y();
		marker.scale.z = moma_param.mesh_parts[index].scale.z();
		marker.mesh_resource = moma_param.mesh_parts[index].file;
		moma_marker.markers.push_back(marker);
	}
	updateMeshMarkers(Eigen::VectorXd::Zero(3 + moma_param.dof_num));
}

double normyaw(const double& yaw)
{
	double y = yaw;
	if (y > M_PI)
		y-=2*M_PI;
	else if (y < -M_PI)
		y+=2*M_PI;
	return y;
}

void rcvStateCallBack(const fake_moma::MomaStatePtr msg)
{
	// chassis
	now_se2[0] = msg->chassis_odom.pose.pose.position.x;
	now_se2[1] = msg->chassis_odom.pose.pose.position.y;
	double ori_z = msg->chassis_odom.pose.pose.orientation.z;
	double ori_w = msg->chassis_odom.pose.pose.orientation.w;
	now_se2[2] = atan2(2.0*ori_z*ori_w, 
						2.0*ori_w*ori_w-1.0);

	// arm
	// now_q = moma_cmd.q.data;
	for (size_t i=0; i<moma_param.dof_num; i++)
		now_q[i] = msg->arm_odom[i].twist.twist.linear.x;
	
	// ################################
	// C++: Update all meshes from the shared typed transform result
	// ################################
	Eigen::VectorXd moma_pos(3 + moma_param.dof_num);
	moma_pos.head(3) = now_se2;
	for (size_t i=0; i<moma_param.dof_num; i++)
	{
		now_q[i] = std::max( moma_param.joint_pos_limit_min[i], 
				   std::min( moma_param.joint_pos_limit_max[i], (double) (now_q[i]) ) );
		moma_pos[3 + i] = now_q[i];
	}
	updateMeshMarkers(moma_pos);

	// collision detection
	Eigen::VectorXi collision_link;
	moma_param.isSelfCollision(moma_pos, collision_link);

	visualization_msgs::MarkerArray cylinder_msg = moma_param.getColliCylinderArray(moma_pos);
	marker_pub.publish(moma_marker);
	cylinder_pub.publish(cylinder_msg);
	auto cma = moma_param.getColliMarkerArray(moma_pos);
	for (size_t i=0; i<cma.markers.size(); i++)
		cma.markers[i].header.stamp = ros::Time::now();
	sphere_pub.publish(cma);
}

int main (int argc, char** argv) 
{        
    ros::init (argc, argv, "moma_vis_node");
    ros::NodeHandle nh( "~" );
    ros::NodeHandle root_nh;

    // ################################
    // C++: Load the shared global /moma robot profile
    // ################################
    moma_param = MomaParam::fromRos(root_nh);
    ROS_INFO_STREAM("[moma_vis] robot_name=" << moma_param.robot_name
                    << " dof_num=" << moma_param.dof_num
                    << " kinematics=" << MomaParam::kinematicsName(moma_param.kinematics)
                    << " mount_t=" << moma_param.relative_t.transpose());

	initParams();
    state_sub  = nh.subscribe("state", 1, rcvStateCallBack);
	marker_pub = nh.advertise<visualization_msgs::MarkerArray>("marker", 1);
	cylinder_pub = nh.advertise<visualization_msgs::MarkerArray>("cylinder", 1);
	sphere_pub = nh.advertise<visualization_msgs::MarkerArray>("sphere", 1);

	ros::spin();
    return 0;
}