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
#include "fake_moma/MomaCmd.h"
#include "fake_moma/moma_param.h"
#include "fake_moma/visual_transform_utils.h"

using namespace std;

ros::Subscriber cmd_sub, map_sub;
ros::Publisher  state_pub, marker_pub, cylinder_pub, sphere_pub, sphere_visual_pub, lidar_odom_pub;

fake_moma::MomaCmd moma_cmd;
fake_moma::MomaState moma_state;
nav_msgs::Odometry lidar_odom;
visualization_msgs::MarkerArray moma_marker;
visualization_msgs::MarkerArray colli_marker;

Eigen::Vector3d now_se2 = Eigen::Vector3d::Zero();
vector<float> now_q;
double time_resolution = 0.01;
bool rcv_cmd = false;
MomaParam moma_param;

bool has_map = false;
pcl::PointCloud<pcl::PointXYZ> cloud_map;
pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree;
pcl::PointXYZ sensor_pose;
vector<int> pointIdxRadiusSearch;
vector<float> pointRadiusSquaredDistance;

vector<Eigen::Vector2d> vw_buff;
double time_delay = 0.4;
// double time_delay = 0.4;
ros::Time get_cmdtime;

double init_x = 0.0;
double init_y = 0.0;
double init_yaw = 0.0;

double lidar_x = 0.0;
double lidar_y = 0.0;
double lidar_z = 0.0;
Eigen::Vector3d lidar_vec;
// ################################
// C++: Resolve AG95 marker by mesh role, not list order
// ################################
int ag95_marker_index = -1;

void updateMeshMarkers(const Eigen::VectorXd& state)
{
	const KinematicResult links = moma_param.getLinkTransforms(state);
	for (size_t index = 0; index < moma_param.mesh_parts.size(); ++index)
	{
		const MeshPart& part = moma_param.mesh_parts[index];
		moma_marker.markers[index].header.stamp = ros::Time::now();
		moma_marker.markers[index].pose = fake_moma_visual::poseFromTransform(
			fake_moma_visual::meshWorldVisualTransform(moma_param, links, part));
	}
}

void initParams(ros::NodeHandle& nh)
{
	nh.getParam("sim/init_x", init_x);
	nh.getParam("sim/init_y", init_y);
	nh.getParam("sim/init_yaw", init_yaw);
	nh.getParam("sim/lidar_x", lidar_x);
	nh.getParam("sim/lidar_y", lidar_y);
	nh.getParam("sim/lidar_z", lidar_z);

	now_se2 = Eigen::Vector3d(init_x, init_y, init_yaw);
	lidar_vec = Eigen::Vector3d(lidar_x, lidar_y, lidar_z);

	//diff_state
	moma_state.chassis_odom.header.frame_id = "world";
	moma_state.chassis_odom.pose.pose.position.x = init_x;
	moma_state.chassis_odom.pose.pose.position.y = init_y;
	moma_state.chassis_odom.pose.pose.position.z = moma_param.chassis_height;
	moma_state.chassis_odom.pose.pose.orientation.w = cos(init_yaw/2.0);
	moma_state.chassis_odom.pose.pose.orientation.x = 0.0;
	moma_state.chassis_odom.pose.pose.orientation.y = 0.0;
	moma_state.chassis_odom.pose.pose.orientation.z = sin(init_yaw/2.0);
	moma_state.chassis_odom.twist.twist.linear.x = 0.0;
	moma_state.chassis_odom.twist.twist.linear.y = 0.0;
	moma_state.chassis_odom.twist.twist.linear.z = 0.0;
	moma_state.chassis_odom.twist.twist.angular.x = 0.0;
	moma_state.chassis_odom.twist.twist.angular.y = 0.0;
	moma_state.chassis_odom.twist.twist.angular.z = 0.0;

	// ################################
	// C++: LiDAR extrinsic relative to planning base_link (no chassis_height / visual root)
	// ################################
	Eigen::Quaterniond chasq(cos(init_yaw/2.0), 0.0, 0.0, sin(init_yaw/2.0));
	Eigen::Vector3d chasv(init_x, init_y, 0.0);
	Eigen::Vector3d lidar_pos = chasq.matrix() * lidar_vec + chasv;
	lidar_odom = moma_state.chassis_odom;
	lidar_odom.pose.pose.position.x = lidar_pos(0);
	lidar_odom.pose.pose.position.y = lidar_pos(1);
	lidar_odom.pose.pose.position.z = lidar_pos(2);

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
		if (moma_param.mesh_parts[index].role == MeshRole::Ag95)
			ag95_marker_index = static_cast<int>(index);
		moma_marker.markers.push_back(marker);
	}
	for (size_t i = 0; i < moma_param.dof_num; ++i)
		moma_state.arm_odom.push_back(moma_state.chassis_odom);
	Eigen::VectorXd state = Eigen::VectorXd::Zero(3 + moma_param.dof_num);
	state.head(3) = now_se2;
	updateMeshMarkers(state);
}

void rcvVelCmdCallBack(const fake_moma::MomaCmd& cmd)
{	
	moma_cmd = cmd;
	moma_cmd.speed = 0.0;
	moma_cmd.angular_velocity = 0.0;
	if (rcv_cmd==false)
	{
		rcv_cmd = true;
		vw_buff.push_back(Eigen::Vector2d(cmd.speed, cmd.angular_velocity));
		get_cmdtime = ros::Time::now();
	}
	else
	{
		vw_buff.push_back(Eigen::Vector2d(cmd.speed, cmd.angular_velocity));
		if ((ros::Time::now() - get_cmdtime).toSec() > time_delay)
		{
			moma_cmd.speed = vw_buff[0](0);
			moma_cmd.angular_velocity = vw_buff[0](1);
			vw_buff.erase(vw_buff.begin());
		}
	}
}

void rcvMapCallBack(const sensor_msgs::PointCloud2& msg)
{	
	if (has_map)
		return;
	pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
	pcl::fromROSMsg(msg, cloud_map);
    kd_tree.setInputCloud(cloud_map.makeShared());
	has_map = true;
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

void simCallBack(const ros::TimerEvent& event)
{
	if (!rcv_cmd)
	{
		moma_state.chassis_odom.header.stamp = ros::Time::now();
    	state_pub.publish(moma_state);
		lidar_odom.header.stamp = ros::Time::now();
		lidar_odom_pub.publish(lidar_odom);
		// ################################
		// C++: Update the base marker without a positional literal
		// ################################
		moma_marker.markers.front().header.stamp = ros::Time::now();
		marker_pub.publish(moma_marker);
		Eigen::VectorXd moma_pos = VectorXd::Zero(3+moma_param.dof_num);
		moma_pos.head(3) = now_se2;
		cylinder_pub.publish(moma_param.getColliCylinderArray(moma_pos));
		auto cma = moma_param.getColliMarkerArray(moma_pos);
		for (size_t i=0; i<cma.markers.size(); i++)
			cma.markers[i].header.stamp = ros::Time::now();
		sphere_pub.publish(cma);
		auto visual_cma = moma_param.getColliVisualMarkerArray(moma_pos);
		for (size_t i = 0; i < visual_cma.markers.size(); ++i)
			visual_cma.markers[i].header.stamp = ros::Time::now();
		sphere_visual_pub.publish(visual_cma);
		return;
	}

	// chassis
	double vx = moma_cmd.speed;
	double wz = moma_cmd.angular_velocity;
	now_se2(0) += vx*time_resolution*cos(now_se2(2));
	now_se2(1) += vx*time_resolution*sin(now_se2(2));
	now_se2(2) += wz*time_resolution;
	now_se2(2) = normyaw(now_se2(2));
	
	moma_state.chassis_odom.header.stamp = ros::Time::now();
	moma_state.chassis_odom.pose.pose.position.x  = now_se2.x();
	moma_state.chassis_odom.pose.pose.position.y  = now_se2.y();
	moma_state.chassis_odom.pose.pose.orientation.w  = cos(now_se2(2)/2);
	moma_state.chassis_odom.pose.pose.orientation.z  = sin(now_se2(2)/2);
	moma_state.chassis_odom.twist.twist.linear.x  = vx;
	moma_state.chassis_odom.twist.twist.angular.z = wz;
	// ################################
	// C++: LiDAR world pose from planning base SE2 * URDF lidar_joint0
	// ################################
	Eigen::Quaterniond chasq(cos(now_se2(2)/2), 0.0, 0.0, sin(now_se2(2)/2));
	Eigen::Vector3d chasv(now_se2.x(), now_se2.y(), 0.0);
	Eigen::Vector3d lidar_pos = chasq.matrix() * lidar_vec + chasv;
	lidar_odom = moma_state.chassis_odom;
	lidar_odom.pose.pose.position.x = lidar_pos(0);
	lidar_odom.pose.pose.position.y = lidar_pos(1);
	lidar_odom.pose.pose.position.z = lidar_pos(2);

	// arm
	// now_q = moma_cmd.q.data;
	for (size_t i=0; i<moma_param.dof_num; i++)
		now_q[i] = moma_cmd.q.data[i];
		// now_q[i] += moma_cmd.dq.data[i]*time_resolution;
	
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
		moma_state.arm_odom[i].twist.twist.linear.x = now_q[i];
		moma_state.arm_odom[i].twist.twist.angular.z = moma_cmd.dq.data[i];
	}
	updateMeshMarkers(moma_pos);
	const KinematicResult links = moma_param.getLinkTransforms(moma_pos);
	for (size_t i = 0; i < moma_param.dof_num; ++i)
		moma_state.arm_odom[i].pose.pose = fake_moma_visual::poseFromTransform(links.arm_link_T[i]);
	if (ag95_marker_index >= 0)
	{
		visualization_msgs::Marker& ag95_marker = moma_marker.markers[ag95_marker_index];
		if (moma_cmd.gripper_state)
		{
			ag95_marker.color.r = ag95_marker.color.g = ag95_marker.color.b = 0.0;
		}
		else
		{
			ag95_marker.color.g = 1.0;
		}
	}

	// collision detection
	Eigen::VectorXi collision_link;
	moma_param.isSelfCollision(moma_pos, collision_link);

	// ################################
	// C++: Debug cylinder KD-tree scan is sim-only; not GridMap planning truth
	// ################################
	visualization_msgs::MarkerArray cylinder_msg = moma_param.getColliCylinderArray(moma_pos);
	for (size_t i=0; i<cylinder_msg.markers.size(); i++)
	{
		pointIdxRadiusSearch.clear();
		pointRadiusSquaredDistance.clear();
		Eigen::Matrix3d cyR = Eigen::Quaterniond(cylinder_msg.markers[i].pose.orientation.w,
												cylinder_msg.markers[i].pose.orientation.x,
												cylinder_msg.markers[i].pose.orientation.y,
												cylinder_msg.markers[i].pose.orientation.z)
												.toRotationMatrix().transpose();
		pcl::PointXYZ pp;
		pp.x = cylinder_msg.markers[i].pose.position.x;
		pp.y = cylinder_msg.markers[i].pose.position.y;
		pp.z = cylinder_msg.markers[i].pose.position.z;
		double scale_x = cylinder_msg.markers[i].scale.x / 2.0;
		double scale_z = cylinder_msg.markers[i].scale.z / 2.0;
		double radius = sqrt(scale_x*scale_x + scale_z*scale_z) * 1.01;
		bool collision = false;
		if (has_map && kd_tree.radiusSearch(pp, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) 
		{
			for (size_t j = 0; j < pointIdxRadiusSearch.size(); j++) 
			{
				pcl::PointXYZ pt = cloud_map.points[pointIdxRadiusSearch[j]];
				Eigen::Vector3d pt_cy = Eigen::Vector3d(pt.x, pt.y, pt.z);
				pt_cy = cyR * (pt_cy - Eigen::Vector3d(pp.x, pp.y, pp.z));
				if (pt_cy[2] > -scale_z && pt_cy[2] < scale_z 
					&& pt_cy.head(2).norm() < scale_x)
				{
					collision = true;
					break;
				}
			}
		}

		// if (!collision && collision_link[i] == 0)
		// {
		// 	moma_marker.markers[i].color.r = 0.5;
		// 	moma_marker.markers[i].color.g = 0.5;
		// 	moma_marker.markers[i].color.b = 0.5;
		// }
		// else
		// {
		// 	if (collision)
		// 		moma_marker.markers[i].color.r = 1.0;
		// 	if (collision_link[i] != 0)
		// 		moma_marker.markers[i].color.b = 1.0;
		// }
	}
	state_pub.publish(moma_state);
	lidar_odom_pub.publish(lidar_odom);
	marker_pub.publish(moma_marker);
	cylinder_pub.publish(cylinder_msg);
	auto cma = moma_param.getColliMarkerArray(moma_pos);
	for (size_t i=0; i<cma.markers.size(); i++)
		cma.markers[i].header.stamp = ros::Time::now();
	sphere_pub.publish(cma);
	auto visual_cma = moma_param.getColliVisualMarkerArray(moma_pos);
	for (size_t i = 0; i < visual_cma.markers.size(); ++i)
		visual_cma.markers[i].header.stamp = ros::Time::now();
	sphere_visual_pub.publish(visual_cma);
}

int main (int argc, char** argv) 
{        
    ros::init (argc, argv, "fake_moma_node");
    ros::NodeHandle nh("~");
    ros::NodeHandle root_nh;

    // ################################
    // C++: Load the shared global /moma robot profile
    // ################################
    moma_param = MomaParam::fromRos(root_nh);
    ROS_INFO_STREAM("[fake_moma] robot_name=" << moma_param.robot_name
                    << " dof_num=" << moma_param.dof_num
                    << " kinematics=" << MomaParam::kinematicsName(moma_param.kinematics)
                    << " mount_t=" << moma_param.relative_t.transpose());
	initParams(nh);
	cmd_sub = nh.subscribe( "command", 1, rcvVelCmdCallBack);
	map_sub = nh.subscribe( "global_map", 1, rcvMapCallBack);
    state_pub  = nh.advertise<fake_moma::MomaState>("state", 1);
	marker_pub = nh.advertise<visualization_msgs::MarkerArray>("marker", 1);
	cylinder_pub = nh.advertise<visualization_msgs::MarkerArray>("cylinder", 1);
	sphere_pub = nh.advertise<visualization_msgs::MarkerArray>("sphere", 1);
	sphere_visual_pub = nh.advertise<visualization_msgs::MarkerArray>("sphere_visual", 1);
	lidar_odom_pub = nh.advertise<nav_msgs::Odometry>("/Odometry", 1);
	ros::Timer odom_timer = nh.createTimer(ros::Duration(time_resolution), simCallBack);

	ros::spin();
    return 0;
}