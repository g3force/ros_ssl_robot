/*
 * joy2vel_xyw.cpp
 *
 *  Created on: Mar 10, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

ros::Publisher pub_vel;

void callbackJoy(const sensor_msgs::Joy::ConstPtr& joy) {
	geometry_msgs::Twist vel;
	vel.linear.x = joy->axes[1];
	vel.linear.y = joy->axes[0];
	vel.angular.z = joy->axes[3];
	pub_vel.publish(vel);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "ssl_robot_joy");

	ros::NodeHandle n;
	pub_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	ros::Subscriber sub_joy = n.subscribe("/joy", 1,
			callbackJoy);

	ros::spin();

	return 0;
}


