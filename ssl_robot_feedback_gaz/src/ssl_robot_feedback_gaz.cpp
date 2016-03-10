/*
 * ssl_robot_feedback_gaz.cpp
 *
 *  Created on: 25.02.2016
 *      Author: NicolaiO
 */

#include "ros/ros.h"

#include <iostream>
#include <fstream>
#include <iomanip>

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

std::ofstream myfile;

int findModel(std::vector<std::string> name) {
	for (int idx = 0; idx < name.size(); idx++) {
		if (name[idx] == "ssl_robot") {
			return idx;
		}
	}
	return -1;
}

double normalizeAngle(double angle) {
	// Don't call this a hack! It's numeric!
	return (angle - (round((angle / (M_PI_2)) - 1e-6) * M_PI_2));
}

double angleDiff(double angle1, double angle2) {
	return normalizeAngle(normalizeAngle(angle1) - normalizeAngle(angle2));
}

double getOrientation(geometry_msgs::Quaternion gq)
{
	double ow = gq.w;
	double ox = gq.x;
	double oy = gq.y;
	double oz = gq.z;

	tf::Quaternion q(ox, oy, oz, ow);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	return yaw;
}

void callbackFilterStates(const nav_msgs::Odometry::ConstPtr& odo) {

	ssl_robot_msgs::State state;
	state.t = ros::Time::now();
	state.global_pos_x = odo->pose.pose.position.x;
	state.global_pos_y = odo->pose.pose.position.y;
	state.global_pos_w = getOrientation(odo->pose.pose.orientation);


//	state.local_vel_x = (state.global_vel_x * cos(-state.global_pos_w))
//			- (state.global_vel_y * sin(-state.global_pos_w));
//	state.local_vel_y = (state.global_vel_y * cos(-state.global_pos_w))
//			+ (state.global_vel_x * sin(-state.global_pos_w));
//	state.local_vel_w = state.global_vel_w;

	tf::TransformListener listener;
//	tf::StampedTransform transform;
//	try {
//		listener.lookupTransform("/ssl_robot_odom", "/ssl_robot", state.t,
//				transform);
//	} catch (tf::TransformException &ex) {
//		ROS_ERROR("%s", ex.what());
//		ros::Duration(1.0).sleep();
//	}

	geometry_msgs::Vector3Stamped lvel;
	lvel.vector = odo->twist.twist.linear;
	lvel.header = odo->header;
	geometry_msgs::Vector3Stamped gvel;
	gvel.vector.x = lvel.vector.x * cos(state.global_pos_w);

//	try {
//		listener.transformVector("vision", lvel, gvel);
//	} catch (tf::TransformException &ex) {
//		ROS_ERROR("%s", ex.what());
//		ros::Duration(1.0).sleep();
//	}

	state.local_vel_x = odo->twist.twist.linear.x;
	state.local_vel_y = odo->twist.twist.linear.y;
	state.local_vel_w = odo->twist.twist.angular.z;

	state.global_vel_x = gvel.vector.x;
	state.global_vel_y = gvel.vector.y;
	state.global_vel_w = state.local_vel_w;

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
	double angle = getOrientation(p.orientation);
	tf::Quaternion q;
	q.setEuler(0, 0, 0);
//	q.setX(p.orientation.x);
//	q.setY(p.orientation.y);
//	q.setZ(p.orientation.z);
//	q.setW(p.orientation.w);
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
	if (lastPose.header.seq != 0) {
		ros::Duration tDiff = pose.header.stamp - lastPose.header.stamp;
		double dt = tDiff.toSec();
		if (dt > 0) {
			raw_vel.twist.linear.x = (pose.pose.pose.position.x
					- lastPose.pose.pose.position.x) / dt;
			raw_vel.twist.linear.y = (pose.pose.pose.position.y
					- lastPose.pose.pose.position.y) / dt;
			raw_vel.twist.angular.z = angleDiff(getOrientation(pose.pose.pose.orientation),
					getOrientation(lastPose.pose.pose.orientation)) / dt;
		} else {
			raw_vel.twist = lastVel.twist;
		}
	}

	myfile << std::fixed << std::setw(11) << std::setprecision(6)
			<< pose.header.stamp.toSec() << " " << pose.pose.pose.position.x
			<< " " << pose.pose.pose.position.y << " "
			<< getOrientation(pose.pose.pose.orientation) << " " << raw_vel.twist.linear.x
			<< " " << raw_vel.twist.linear.y << " " << raw_vel.twist.angular.z
			<< std::endl;

	lastPose = pose;
	lastVel = raw_vel;

//	pub_raw_vel.publish(raw_vel);
//	ros::spinOnce();
	pub_raw_state.publish(pose);
	ros::spinOnce();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "ssl_robot_feedback_gaz");
	ros::NodeHandle n;

	ros::Time time;
	do {
		time = ros::Time::now();
	} while (time.isZero());

	myfile.open("/tmp/gazebo.csv");

	ros::Subscriber sub_model_states = n.subscribe("/gazebo/model_states", 1,
			callbackModelStates);

	ros::Subscriber sub_filter_states = n.subscribe("/ssl_robot/filtered_state",
			1, callbackFilterStates);

	pub_raw_state = n.advertise<geometry_msgs::PoseWithCovarianceStamped>(
			"/ssl_robot/raw_state", 1);
//	pub_raw_vel = n.advertise<geometry_msgs::TwistStamped>("/ssl_robot/raw_vel",
//			1);
	pub_state = n.advertise<ssl_robot_msgs::State>("/ssl_robot/state", 1);

	ros::spin();

	myfile.close();

	return 0;
}

