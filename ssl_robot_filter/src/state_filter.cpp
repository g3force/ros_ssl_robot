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


void callbackOdom(const nav_msgs::Odometry::ConstPtr& odo) {

	// definitions
	ros::Time time = odo->header.stamp;
	geometry_msgs::Pose p = odo->pose.pose;
	geometry_msgs::Twist v = odo->twist.twist;

	// send new transform
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(p.position.x, p.position.y, 0.0));
	double angle = getOrientation(p.orientation);
	tf::Quaternion q = tf::createQuaternionFromYaw(angle);
//	q.setRPY(0, 0, angle);
//	q.setRPY(0,0,0);
	transform.setRotation(q);
	br.sendTransform(
			tf::StampedTransform(transform, time, "odom",
					"base_link"));

	// convert pose
	geometry_msgs::PoseWithCovarianceStamped pose;
	pose.header.frame_id = "map";
	pose.header.stamp = time;
	pose.header.seq = seq++;
	pose.pose.pose = p;
	pose.pose.covariance.elems[0] = 0.0001;
	pose.pose.covariance.elems[6 + 1] = 0.0001;
	pose.pose.covariance.elems[5 * 6 + 5] = 0.0005;
	pub_raw_pose.publish(pose);

	// convert twist
	geometry_msgs::TwistWithCovarianceStamped twist;
	twist.header = pose.header;
	twist.header.frame_id = "base_link";

	if (lastPose.header.seq != 0) {
		ros::Duration tDiff = pose.header.stamp - lastPose.header.stamp;
		double dt = tDiff.toSec();
		if (dt > 0.001) {
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

		twist.twist.covariance.elems[0] = 0.001;
		twist.twist.covariance.elems[6 + 1] = 0.001;
		twist.twist.covariance.elems[5 * 6 + 5] = 0.001;
	} else {
		lastPose = pose;
		lastTwist = twist;
	}

	pub_raw_twist.publish(twist);

	// send raw data as state
	nav_msgs::Odometry odom;
	odom.header = twist.header;
	odom.child_frame_id = "odom";
	odom.pose = pose.pose;
	odom.twist = twist.twist;
	pub_state.publish(odom);

	ros::spinOnce();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "state_filter");
	ros::NodeHandle n;

	ros::Time time;
	do {
		time = ros::Time::now();
	} while (time.isZero());

	ros::Subscriber sub_model_states = n.subscribe("state", 1, callbackOdom);
	pub_state = n.advertise<nav_msgs::Odometry>("/filter/state", 1);

	ros::spin();

	return 0;
}

