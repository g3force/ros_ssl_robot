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
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "gazebo_msgs/ModelStates.h"
#include "ssl_robot_msgs/State.h"

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <angles/angles.h>

#include "AngleMath.h"

ros::Publisher pub_state;
ros::Publisher pub_raw_pose;
ros::Publisher pub_raw_twist;

geometry_msgs::PoseWithCovarianceStamped lastPose;
geometry_msgs::TwistWithCovarianceStamped lastTwist;
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
	nav_msgs::Odometry state = *odo;
	pub_state.publish(state);
	ros::spinOnce();
}

void callbackModelStates(const gazebo_msgs::ModelStates::ConstPtr& states) {
	// get model id
	int idx = findModel(states->name);
	if (idx < 0)
		return;

	// definitions
	ros::Time time = ros::Time::now();
	geometry_msgs::Pose p = states->pose[idx];
	geometry_msgs::Twist v = states->twist[idx];

	// send new transform
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(p.position.x, p.position.y, 0.0));
	double angle = getOrientation(p.orientation);
	tf::Quaternion q;
//	q.setRPY(0, 0, angle);
	q.setRPY(0,0,0);
	transform.setRotation(q);
	br.sendTransform(
			tf::StampedTransform(transform, time, "odom",
					"base_link"));

	// convert pose
	geometry_msgs::PoseWithCovarianceStamped pose;
	pose.header.frame_id = "vision";
	pose.header.stamp = time;
	pose.header.seq = seq++;
	pose.pose.pose = p;
	pose.pose.covariance.elems[0] = 0.0001;
	pose.pose.covariance.elems[6 + 1] = 0.0001;
	pose.pose.covariance.elems[5 * 6 + 5] = 0.0005;
	pub_raw_pose.publish(pose);
	ros::spinOnce();

	// convert twist
	geometry_msgs::TwistWithCovarianceStamped twist;
	twist.header = pose.header;
	twist.header.frame_id = "base_link";

	if (lastPose.header.seq != 0) {
		ros::Duration tDiff = pose.header.stamp - lastPose.header.stamp;
		double dt = tDiff.toSec();
		if (dt > 0) {
			double vx = (pose.pose.pose.position.x
					- lastPose.pose.pose.position.x) / dt;
			double vy = (pose.pose.pose.position.y
					- lastPose.pose.pose.position.y) / dt;

			double angle = -getOrientation(pose.pose.pose.orientation);
			twist.twist.twist.linear.x = vx * cos(angle) - vy * sin(angle);
			twist.twist.twist.linear.y = vy * cos(angle) + vx * sin(angle);
			twist.twist.twist.angular.z = angleDiff(getOrientation(pose.pose.pose.orientation),
					getOrientation(lastPose.pose.pose.orientation)) / dt;

			lastPose = pose;
			lastTwist = twist;
		} else {
			twist.twist = lastTwist.twist;
		}

		twist.twist.covariance.elems[0] = 0.1;
		twist.twist.covariance.elems[6 + 1] = 0.1;
		twist.twist.covariance.elems[5 * 6 + 5] = 0.1;
	} else {
		lastPose = pose;
		lastTwist = twist;
	}

	pub_raw_twist.publish(twist);
	ros::spinOnce();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "ssl_robot_feedback_gaz");
	ros::NodeHandle n;

	ros::Time time;
	do {
		time = ros::Time::now();
	} while (time.isZero());

	ros::Subscriber sub_model_states = n.subscribe("/gazebo/model_states", 1,
			callbackModelStates);
	ros::Subscriber sub_filter_states = n.subscribe("/ssl_robot/filtered_state",
			1, callbackFilterStates);

	pub_raw_pose = n.advertise<geometry_msgs::PoseWithCovarianceStamped>(
			"/ssl_robot/raw_pose", 1);
	pub_raw_twist = n.advertise<geometry_msgs::TwistWithCovarianceStamped>(
			"/ssl_robot/raw_twist", 1);
	pub_state = n.advertise<nav_msgs::Odometry>("/ssl_robot/state", 1);

	ros::spin();

	return 0;
}

