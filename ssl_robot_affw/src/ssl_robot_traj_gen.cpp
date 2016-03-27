/*
 * traj-gen.cpp
 *
 *  Created on: 24.02.2016
 *      Author: NicolaiO
 */

#include <affw_ctrl/ActionRequest.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/service_client.h>
#include <ros/time.h>
#include <std_msgs/Header.h>
#include <iostream>
#include <fstream>
#include <iterator>
#include <vector>

int main(int argc, char **argv) {
	ros::init(argc, argv, "ssl_robot_traj_gen");
	ros::NodeHandle n;
	ros::Publisher pub_set_vel = n.advertise<geometry_msgs::TwistStamped>(
			"/ssl_robot_affw/target_vel", 1);

	if(argc != 2)
	{
		std::cout << "Usage: " << argv[0] << " <traj-file>" << std::endl;
		return 1;
	}

	ros::ServiceClient srv_action;
	srv_action = n.serviceClient<affw_ctrl::ActionRequest>("/affw_ctrl/action");
	bool avail = srv_action.waitForExistence();

	if(!avail)
		return 1;

	const char* filename = argv[1];

	std::vector<geometry_msgs::TwistStamped> setpoints;
	std::ifstream file(filename);
	while(ros::Time::now().isZero());
	ros::Time now = ros::Time::now();
	double t, vx, vy, vw;
	while (file >> t >> vx >> vy >> vw) {
		geometry_msgs::TwistStamped vel;
		vel.header.stamp = now + ros::Duration(t);
		vel.header.frame_id = "base_link";
		vel.twist.linear.x = vx;
		vel.twist.linear.y = vy;
		vel.twist.angular.z = vw;
		setpoints.push_back(vel);
	}

	std::vector<geometry_msgs::TwistStamped>::iterator it = setpoints.begin();
	do {
		pub_set_vel.publish(*it);
		std::cout << it->header.stamp << " " << it->twist.linear.x << " " << it->twist.linear.y << " " << it->twist.angular.z << std::endl;
		ros::Time t1 = it->header.stamp;
		it++;
		if(it < setpoints.end())
		{
			ros::Time t2 = it->header.stamp;
			ros::Duration diff = t2-t1;
			diff.sleep();
		}
		else
		{
			break;
		}
	} while(ros::ok());

	geometry_msgs::TwistStamped vel;
	vel.header.stamp = now + ros::Duration(t);
	vel.header.frame_id = "base_link";
	pub_set_vel.publish(vel);

	ros::shutdown();

	return 0;
}

