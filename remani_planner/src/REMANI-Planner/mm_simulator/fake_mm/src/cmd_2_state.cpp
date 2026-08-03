#include <control_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <iostream>
#include <string>
#include <utility>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <mm_config/mm_config.hpp>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

int _manipulator_dof;

ros::Subscriber car_cmd_sub;
ros::Subscriber joint_cmd_sub, gripper_cmd_sub;
ros::Publisher  car_odom_pub;
ros::Publisher  joint_state_pub, gripper_state_pub, gripper_angle_pub;
// ################################
// C++: URDF joint_state publisher globals begin
// ################################
ros::Publisher  urdf_joint_state_pub_;
bool pub_urdf_joint_states_ = false;
std::vector<std::string> urdf_joint_names_;
// ################################
// C++: URDF joint_state publisher globals end
// ################################

double _init_x, _init_y, _init_yaw;
Eigen::VectorXd _init_theta;

bool rcv_car_cmd   = false;
bool rcv_joint_cmd = false;
bool gripper_state_ = false; // false: open true: close
double gripper_angle_ = M_PI / 6; // M_PI / 6: open 0: close
bool rcv_gripper_state_ = false;
geometry_msgs::Twist _car_vel_cmd;
control_msgs::JointTrajectoryControllerState _joint_pos_cmd;
nav_msgs::Odometry _last_odom;
sensor_msgs::JointState _last_joint_state;
double _last_yaw;

double _time_resolution = 0.001;

ros::Time last_cmd_time_ = ros::Time(0);

void rcvCarVelCmdCallBack(const geometry_msgs::Twist cmd){	
	rcv_car_cmd  = true;
	_car_vel_cmd = cmd;
	last_cmd_time_ = ros::Time::now();
}

void rcvJointCmdCallBack(const control_msgs::JointTrajectoryControllerState cmd){ // pos cmd
	rcv_joint_cmd = true;
	_joint_pos_cmd = cmd;
	last_cmd_time_ = ros::Time::now();
}

void rcvGripperCmdCallBack(const std_msgs::Bool cmd){
	if(!rcv_gripper_state_){
		rcv_gripper_state_ = true;
		if(cmd.data){
			gripper_angle_ = 0;
		}else{
			gripper_angle_ = M_PI / 6;
		}
	}
	gripper_state_ = cmd.data;
}

void normyaw(double& y){
	if (y >= M_PI){
		y -= 2 * M_PI;
	}
	else if (y < -M_PI){
		y += 2 * M_PI;
	}
}

void pubCarOdom(){
	nav_msgs::Odometry odom;
	odom.header.stamp    = ros::Time::now();
	odom.header.frame_id = "world";
	double yaw;

	if(rcv_car_cmd){
		// true val
		odom.pose.pose.position.x = _car_vel_cmd.linear.x;
		odom.pose.pose.position.y = _car_vel_cmd.linear.y;
		odom.pose.pose.position.z = 0.0;
		yaw = _car_vel_cmd.linear.z;
		normyaw(yaw);
		odom.twist.twist.linear.x = _car_vel_cmd.angular.x;
		odom.twist.twist.linear.y = _car_vel_cmd.angular.y;
		odom.twist.twist.linear.z = 0.0;

		odom.pose.pose.orientation.w = cos(yaw / 2);
		odom.pose.pose.orientation.x = 0.0;
		odom.pose.pose.orientation.y = 0.0;
		odom.pose.pose.orientation.z = sin(yaw / 2);

		odom.twist.twist.angular.x = 0.0;
		odom.twist.twist.angular.y = 0.0;
		odom.twist.twist.angular.z = 0.0;
	}else{
		odom.pose.pose.position.x = _init_x;
	    odom.pose.pose.position.y = _init_y;
	    odom.pose.pose.position.z = 0.0;

	    odom.pose.pose.orientation.w = cos(_init_yaw / 2);
	    odom.pose.pose.orientation.x = 0;
	    odom.pose.pose.orientation.y = 0;
	    odom.pose.pose.orientation.z = sin(_init_yaw / 2);

	    odom.twist.twist.linear.x = 0.0;
	    odom.twist.twist.linear.y = 0.0;
	    odom.twist.twist.linear.z = 0.0;

	    odom.twist.twist.angular.x = 0.0;
	    odom.twist.twist.angular.y = 0.0;
	    odom.twist.twist.angular.z = 0.0;

		yaw = _init_yaw;
	}

    car_odom_pub.publish(odom);
	_last_odom = odom;
	_last_yaw = yaw;

	// ################################
	// C++: broadcast world -> base_link for URDF sim begin
	// ################################
	if(pub_urdf_joint_states_){
		static tf::TransformBroadcaster br;
		tf::Transform transform;
		transform.setOrigin(tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, 0.0));
		tf::Quaternion q(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
		                 odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, odom.header.stamp, "world", "base_link"));
	}
	// ################################
	// C++: broadcast world -> base_link for URDF sim end
	// ################################
}

void pubJointState(){ // pos cmd
	sensor_msgs::JointState joint_state;
	joint_state.header.stamp = ros::Time::now();
	joint_state.header.frame_id = "world";
	std::vector<double> joint_p, joint_v, joint_effort;
	joint_p.clear();
	joint_v.clear();
	joint_effort.clear();
	if(rcv_joint_cmd){
		for(int i = 0; i < _manipulator_dof; ++i){
			joint_effort.push_back(_joint_pos_cmd.desired.effort[i]);
			joint_v.push_back(_joint_pos_cmd.desired.velocities[i]);
			joint_p.push_back(_joint_pos_cmd.desired.positions[i]);
		}
	}else{
		for(int i = 0; i < _manipulator_dof; ++i){
			joint_p.push_back(_init_theta(i));
			joint_v.push_back(0.0);
			joint_effort.push_back(0.0);
		}
	}
	// ################################
	// C++: CR10 joint names + ensure size-6 velocity begin
	// ################################
	joint_state.name.clear();
	for(int i = 0; i < _manipulator_dof; ++i){
		joint_state.name.push_back("cr10_joint" + std::to_string(i + 1));
	}
	if((int)joint_p.size() < _manipulator_dof){
		joint_p.resize(_manipulator_dof, 0.0);
	}
	if((int)joint_v.size() < _manipulator_dof){
		joint_v.resize(_manipulator_dof, 0.0);
	}
	if((int)joint_effort.size() < _manipulator_dof){
		joint_effort.resize(_manipulator_dof, 0.0);
	}
	// ################################
	// C++: CR10 joint names + ensure size-6 velocity end
	// ################################
	joint_state.position = joint_p;
	joint_state.velocity = joint_v;
	joint_state.effort = joint_effort;
	_last_joint_state = joint_state;
	joint_state_pub.publish(joint_state);

	// ################################
	// C++: combined /joint_states for robot_state_publisher begin
	// ################################
	if(pub_urdf_joint_states_){
		sensor_msgs::JointState urdf_js;
		urdf_js.header = joint_state.header;
		urdf_js.name = urdf_joint_names_;
		urdf_js.position.assign(urdf_joint_names_.size(), 0.0);
		urdf_js.velocity.assign(urdf_joint_names_.size(), 0.0);
		for(size_t i = 0; i < urdf_joint_names_.size(); ++i){
			for(int j = 0; j < _manipulator_dof; ++j){
				if(urdf_joint_names_[i] == joint_state.name[j]){
					urdf_js.position[i] = joint_state.position[j];
					urdf_js.velocity[i] = joint_state.velocity[j];
					break;
				}
			}
		}
		urdf_joint_state_pub_.publish(urdf_js);
	}
	// ################################
	// C++: combined /joint_states for robot_state_publisher end
	// ################################

	std_msgs::Bool gripper_state;
	gripper_state.data = gripper_state_;
	gripper_state_pub.publish(gripper_state);
}

int main (int argc, char** argv) 
{        
    ros::init (argc, argv, "odom_generator");
    ros::NodeHandle nh( "~" );

	nh.param("mm/manipulator_dof", _manipulator_dof, -1);
	
	std::vector<double> init_state;
    nh.getParam("fsm/init_state", init_state);
	_init_x = init_state[0];
	_init_y = init_state[1];
	nh.param("fsm/init_yaw", _init_yaw,  0.0);
	_init_yaw = _init_yaw / 180.0 * M_PI;
	_init_theta = Eigen::VectorXd::Zero(_manipulator_dof);
	for(int i = 0; i < _manipulator_dof; i++){
    	_init_theta(i) = init_state[i + 2] * M_PI / 180.0;
    }
	nh.param("fsm/waypoint0_gripper_close", gripper_state_,  false);

	// ################################
	// C++: optional full URDF joint_states for RSP begin
	// ################################
	nh.param("fake_mm/pub_urdf_joint_states", pub_urdf_joint_states_, false);
	urdf_joint_names_ = {
		"fr_steering_joint", "fr_wheel_joint",
		"fl_steering_wheel_joint", "fl_wheel_joint",
		"rl_steering_wheel_joint", "rl_wheel_joint",
		"rr_steering_wheel_joint", "rr_wheel_joint",
		"cr10_joint1", "cr10_joint2", "cr10_joint3",
		"cr10_joint4", "cr10_joint5", "cr10_joint6",
		"gripper_finger1_joint", "gripper_finger2_joint"
	};
	if(pub_urdf_joint_states_){
		urdf_joint_state_pub_ = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
	}
	// ################################
	// C++: optional full URDF joint_states for RSP end
	// ################################

	std::vector<double> joint_p, joint_v, joint_effort;
	joint_p.clear();
	joint_v.clear();
	joint_effort.clear();
	for(int i = 0; i < _manipulator_dof; ++i){
		joint_p.push_back(_init_theta(i));
		joint_v.push_back(0.0);
		joint_effort.push_back(0.0);
	}
	_last_joint_state.position = joint_p;
	_last_joint_state.velocity = joint_v;
	_last_joint_state.effort = joint_effort;

	car_cmd_sub     = nh.subscribe("/mm_controller_node/car_cmd", 1, rcvCarVelCmdCallBack);
	joint_cmd_sub   = nh.subscribe("/mm_controller_node/joint_cmd", 1, rcvJointCmdCallBack);
	gripper_cmd_sub   = nh.subscribe("gripper_cmd", 1, rcvGripperCmdCallBack);
	car_odom_pub    = nh.advertise<nav_msgs::Odometry>("odometry", 1);
	joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_state", 1);
	gripper_state_pub = nh.advertise<std_msgs::Bool>("gripper_state", 1);
	gripper_angle_pub = nh.advertise<std_msgs::Float64>("gripper_angle", 1);

    ros::Rate rate(1.0 / _time_resolution);
    bool status = ros::ok();
    while(status){
		pubCarOdom();
		pubJointState();
        ros::spinOnce();
        status = ros::ok();
        rate.sleep();
    }

    return 0;
}