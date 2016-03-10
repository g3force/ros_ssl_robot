/*
 * ssl_robot_feedback_gaz.cpp
 *
 *  Created on: 25.02.2016
 *      Author: NicolaiO
 */

#include "ros/ros.h"

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "gazebo_msgs/ModelStates.h"
#include "ssl_robot_msgs/State.h"

#include <tf/transform_datatypes.h>
#include <angles/angles.h>

ros::Publisher pub_state;
ros::Publisher pub_raw_state;
ros::Publisher pub_raw_vel;

geometry_msgs::PoseWithCovarianceStamped lastPose;
geometry_msgs::TwistStamped lastVel;
long seq = 0;

int findModel(std::vector<std::string> name) {
	for (int idx = 0; idx < name.size(); idx++) {
		if (name[idx] == "ssl_robot") {
			return idx;
		}
	}
	return -1;
}

void callbackFilterStates(const nav_msgs::Odometry::ConstPtr& odo) {

	ssl_robot_msgs::State state;
	state.t = ros::Time::now();
	state.global_pos_x = odo->pose.pose.position.x;
	state.global_pos_y = odo->pose.pose.position.y;
	double ow = odo->pose.pose.orientation.w;
	double ox = odo->pose.pose.orientation.x;
	double oy = odo->pose.pose.orientation.y;
	double oz = odo->pose.pose.orientation.z;

	state.global_vel_x = odo->twist.twist.linear.x;
	state.global_vel_y = odo->twist.twist.linear.y;
	state.global_vel_w = odo->twist.twist.angular.z;

	tf::Quaternion q(ox, oy, oz, ow);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	state.global_pos_w = yaw;

	state.local_vel_x = (state.global_vel_x * cos(-state.global_pos_w)) - (state.global_vel_y * sin(-state.global_pos_w));
	state.local_vel_y = (state.global_vel_y * cos(-state.global_pos_w)) + (state.global_vel_x * sin(-state.global_pos_w));
	state.local_vel_w = state.global_vel_w;

	tf::TransformListener listener;
//	tf::StampedTransform transform;
//	try {
//		listener.lookupTransform("/ssl_robot_odom", "/ssl_robot", state.t,
//				transform);
//	} catch (tf::TransformException &ex) {
//		ROS_ERROR("%s", ex.what());
//		ros::Duration(1.0).sleep();
//	}


//	geometry_msgs::Vector3Stamped gvel;
//	gvel.vector = odo->twist.twist.linear;
//	gvel.header = odo->header;
//	geometry_msgs::Vector3Stamped lvel;
//	try {
//		listener.transformVector("/ssl_robot", gvel, lvel);
//	} catch (tf::TransformException &ex) {
//		ROS_ERROR("%s", ex.what());
//		ros::Duration(1.0).sleep();
//	}

//	state.local_vel_x = lvel.vector.x;
//	state.local_vel_y = lvel.vector.y;
//	state.local_vel_w = state.global_vel_w;

	pub_state.publish(state);
	ros::spinOnce();
}
void callbackModelStates(const gazebo_msgs::ModelStates::ConstPtr& states) {
	int idx = findModel(states->name);
	if (idx < 0)
		return;

	ros::Time time = ros::Time::now();

	geometry_msgs::Pose p = states->pose[idx];

	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(p.position.x, p.position.y, 0.0));
	tf::Quaternion q;
	q.setX(p.orientation.x);
	q.setY(p.orientation.y);
	q.setZ(p.orientation.z);
	q.setW(p.orientation.w);
	q = q.inverse();
//	q.setRPY(0,0,0);
	transform.setRotation(q);
	br.sendTransform(
			tf::StampedTransform(transform, time, "ssl_robot_odom",
					"ssl_robot"));

	geometry_msgs::PoseWithCovarianceStamped pose;
	pose.header.frame_id = "vision";
	pose.header.stamp = time;
	pose.header.seq = seq++;
	pose.pose.pose = p;
	pose.pose.covariance.elems[0] = 0.01;
	pose.pose.covariance.elems[6 + 1] = 0.01;
	pose.pose.covariance.elems[5 * 6 + 5] = 0.1;

	geometry_msgs::TwistStamped raw_vel;
	raw_vel.header = pose.header;
	if(lastPose.header.seq != 0)
	{
		ros::Duration tDiff = pose.header.stamp - lastPose.header.stamp;
		double dt = tDiff.toSec();
		if(dt > 0)
		{
			raw_vel.twist.linear.x = (pose.pose.pose.position.x - lastPose.pose.pose.position.x) / dt;
			raw_vel.twist.linear.y = (pose.pose.pose.position.y - lastPose.pose.pose.position.y) / dt;
			// TODO normalize
			raw_vel.twist.angular.z = (pose.pose.pose.orientation.z - lastPose.pose.pose.orientation.z) / dt;
		} else {
			raw_vel.twist = lastVel.twist;
		}
	}

	lastPose = pose;
	lastVel = raw_vel;

	pub_raw_vel.publish(raw_vel);
	pub_raw_state.publish(pose);
	ros::spinOnce();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "ssl_robot_feedback_gaz");
	ros::NodeHandle n;

	ros::Subscriber sub_model_states = n.subscribe("/gazebo/model_states", 1,
			callbackModelStates);

	ros::Subscriber sub_filter_states = n.subscribe("/ssl_robot/filtered_state",
			1, callbackFilterStates);

	pub_raw_state = n.advertise<geometry_msgs::PoseWithCovarianceStamped>(
			"/ssl_robot/raw_state", 1);
	pub_raw_vel = n.advertise<geometry_msgs::TwistStamped>(
			"/ssl_robot/raw_vel", 1);
	pub_state = n.advertise<ssl_robot_msgs::State>("/ssl_robot/state", 1);

	ros::spin();

	return 0;
}

